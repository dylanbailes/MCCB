/*
 * main.c - STM32G431CBU6 Helmholtz Coil Controller
 * Unified firmware: dual DRV5055A1 hall sensing + INA240A1 current sense +
 * V_BUS monitoring + DRV8874 PWM drive.
 *
 * Raw registers only — no HAL. 16 MHz HSI.
 *
 * ── ADC2 channel map ────────────────────────────────────────────────────────
 *  PA0 = ADC2_IN1  ISENSE_OUT  (INA240A1, 50mΩ shunt, gain 20 → 1V/A)
 *  PA1 = ADC2_IN2  Hall S1     (DRV5055A1, 60mV/mT, ratiometric)
 *  PA6 = ADC2_IN3  Hall S2     (DRV5055A1, 60mV/mT, ratiometric)
 *  PA7 = ADC2_IN4  V_BUS_MON  (100kΩ/10kΩ divider → ÷11, reads up to ~36V)
 *
 * ── UART ────────────────────────────────────────────────────────────────────
 *  PA2 = USART2 TX (AF7) → Pi GPIO15 (pin 10)
 *  PA3 = USART2 RX (AF7) → Pi GPIO14 (pin 8)
 *  9600 baud, 16 MHz HSI
 *
 * ── PWM (TIM1 on APB2) ──────────────────────────────────────────────────────
 *  PA8 = TIM1_CH1 (AF6) → DRV8874 IN1
 *  PA9 = TIM1_CH2 (AF6) → DRV8874 IN2
 *  16 kHz PWM: PSC=0, ARR=999, 16 MHz HSI
 *  TIM1->BDTR |= TIM_BDTR_MOE is mandatory (advanced timer).
 *
 * ── DRV8874 IN/IN mode (PHEN tied to GND) ───────────────────────────────────
 *  IN1=PWM  IN2=0   → Forward
 *  IN1=0    IN2=PWM → Reverse
 *  IN1=1    IN2=1   → Brake
 *  IN1=0    IN2=0   → Coast
 *
 * ── DRV8874 Status Pins ─────────────────────────────────────────────────────
 *  PB6 = nSLEEP (Output, Active Low) → HIGH = Awake, LOW = Sleep
 *  PB7 = nFAULT (Input, Active Low)  → HIGH = OK, LOW = Fault (Overcurrent/Temp)
 *
 * ── DRV5055A1 hall sensors ──────────────────────────────────────────────────
 *  Sensitivity: 60 mV/mT @ 3.3V, ratiometric (midscale = VCC/2)
 *  Range: ±22 mT. Boot calibration removes per-sensor DC offset.
 *  AC RMS computed over RMS_SAMPLES for 0–250 Hz field measurement.
 *
 * ── INA240A1 current sense ──────────────────────────────────────────────────
 *  Gain: 20 V/V. Shunt: 50mΩ (WSL2512R0500FEA) → 1.0 V/A
 *  Bidirectional: VREF = VCC/2 → centred at 2048 counts at zero current.
 *  Resolution: ~0.8 mA/count. Range: ±3.3A before ADC clips.
 *  I_mA = (raw - zero_isense) * 3300 / 4096
 *
 * ── V_BUS_MON ───────────────────────────────────────────────────────────────
 *  R5=100kΩ (to +24V_BUS), R4=10kΩ (to GND) → ratio 1/11
 *  V_BUS_mV = raw * 36300 / 4096
 *  At 24V: V_ADC ≈ 2.18V → raw ≈ 2703
 *
 * ── UART commands ───────────────────────────────────────────────────────────
 *  'R' = single full report (hall + current + VBUS + PWM + DRV state)
 *  'A' = toggle auto-report (~200 ms interval)
 *  'S' = stream all four raw ADC channels until any keypress
 *  'Z' = re-run zero calibration (zero current + zero field required)
 *  'F' = forward at current duty cycle
 *  'V' = reverse at current duty cycle   (not 'R' — avoids report conflict)
 *  'B' = brake
 *  'C' = coast
 *  '+' = duty +10% (clamped to 100%)
 *  '-' = duty -10% (clamped to 0%)
 *  'P' = print PWM + DRV state
 *  'D' = print DRV state only
 *  'N' = toggle nSLEEP (clears latched faults)
 */

#include "stm32g4xx.h"

/* ─── Hall sensor constants ──────────────────────────────────────────────── */
#define SENSITIVITY_NUM       330000UL  /* 3300mV × 100 for ×100 fixed-point  */
#define SENSITIVITY_DEN       245760UL  /* 4096 counts × 60 mV/mT             */
#define NOMINAL_MIDSCALE      2048
#define LINEAR_RANGE_MT_X100  2200      /* ±22.00 mT saturation limit          */
#define CAL_SAMPLES           512
#define RMS_SAMPLES           512

/* ─── PWM constants ──────────────────────────────────────────────────────── */
#define PWM_ARR               999       /* ARR for 16 kHz at 16 MHz HSI        */
#define PWM_DUTY_STEP         100       /* 10% of ARR=999 per +/- step         */
#define PWM_MAX               PWM_ARR
#define PWM_MIN               0

/* ─── RX circular buffer ─────────────────────────────────────────────────── */
#define RX_BUF_SIZE 16
static volatile char     rx_buf[RX_BUF_SIZE];
static volatile uint8_t  rx_head = 0;
static volatile uint8_t  rx_tail = 0;

void USART2_IRQHandler(void) {
    if (USART2->ISR & USART_ISR_RXNE) {
        rx_buf[rx_head] = (char)USART2->RDR;
        rx_head = (rx_head + 1) % RX_BUF_SIZE;
    }
}

static inline uint8_t rx_available(void) {
    return rx_head != rx_tail;
}

static inline char rx_getc(void) {
    char c = rx_buf[rx_tail];
    rx_tail = (rx_tail + 1) % RX_BUF_SIZE;
    return c;
}

typedef enum {
    DRV_COAST   = 0,
    DRV_FORWARD = 1,
    DRV_REVERSE = 2,
    DRV_BRAKE   = 3,
} drv_mode_t;

/* ─── State ──────────────────────────────────────────────────────────────── */
static int32_t  zero_count[2] = { NOMINAL_MIDSCALE, NOMINAL_MIDSCALE };
static int32_t  zero_isense   = 2048;
static uint32_t pwm_duty      = 0;      /* 0–999 (maps to 0–100% of ARR)      */
static drv_mode_t drv_mode    = DRV_COAST;

/* ─── UART ───────────────────────────────────────────────────────────────── */

static void uart_putc(char c) {
    while (!(USART2->ISR & USART_ISR_TXE));
    USART2->TDR = (uint8_t)c;
}

static void uart_puts(const char *s) {
    while (*s) uart_putc(*s++);
}

static void uart_putn(uint32_t n) {
    char buf[12];
    int i = 0;
    if (n == 0) { uart_putc('0'); return; }
    while (n > 0) { buf[i++] = '0' + (char)(n % 10); n /= 10; }
    while (i--) uart_putc(buf[i]);
}

/* Print signed ×100 fixed-point as "X.XX" */
static void uart_put_fixed2(int32_t val_x100) {
    if (val_x100 < 0) { uart_putc('-'); val_x100 = -val_x100; }
    uart_putn((uint32_t)(val_x100 / 100));
    uart_putc('.');
    uint32_t frac = (uint32_t)(val_x100 % 100);
    if (frac < 10) uart_putc('0');
    uart_putn(frac);
}

/* ─── ADC ────────────────────────────────────────────────────────────────── */

/*
 * Single software-triggered conversion on ADC2.
 * Channels: 1=PA0(ISENSE), 2=PA1(S1), 3=PA6(S2), 4=PA7(VBUS)
 * SMPR1 covers ch1–9, 3 bits each: shift = ch * 3
 */
static inline uint32_t adc2_read(uint32_t ch) {
    ADC2->SQR1  = (ch << 6);
    ADC2->SMPR1 = (7U << (ch * 3));    /* max sampling time */
    ADC2->CFGR  = 0;                   /* single, software trigger */
    ADC2->CR   |= ADC_CR_ADSTART;
    while (!(ADC2->ISR & ADC_ISR_EOC));
    return ADC2->DR;
}

/* ─── Math helpers ───────────────────────────────────────────────────────── */

static uint32_t isqrt(uint32_t n) {
    if (n == 0) return 0;
    uint32_t x = n, y = (x + 1) / 2;
    while (y < x) { x = y; y = (x + n / x) / 2; }
    return x;
}

/* ─── Conversions ────────────────────────────────────────────────────────── */

/* raw → mT × 100, zeroed per-sensor */
static int32_t raw_to_mt_x100(uint32_t raw, int sensor) {
    int32_t delta = (int32_t)raw - zero_count[sensor];
    return (delta * (int32_t)SENSITIVITY_NUM) / (int32_t)SENSITIVITY_DEN;
}

/* raw → current in mA (signed, bidirectional, zeroed) */
static int32_t raw_to_ma(uint32_t raw) {
    int32_t delta = (int32_t)raw - zero_isense;
    return (delta * 3300L) / 4096L;
}

/* raw → bus voltage in mV (÷11 divider) */
static uint32_t raw_to_vbus_mv(uint32_t raw) {
    return (raw * 36300UL) / 4096UL;
}

/* AC RMS in mT × 100, DC-zeroed per-sensor */
static uint32_t ac_rms_mt_x100(uint32_t ch, int sensor) {
    uint64_t sum_sq = 0;
    for (int i = 0; i < RMS_SAMPLES; i++) {
        int32_t delta = (int32_t)adc2_read(ch) - zero_count[sensor];
        sum_sq += (uint64_t)((int64_t)delta * delta);
    }
    uint32_t rms_counts = isqrt((uint32_t)(sum_sq / RMS_SAMPLES));
    return (rms_counts * (uint32_t)SENSITIVITY_NUM) / (uint32_t)SENSITIVITY_DEN;
}

/* True RMS of raw counts (for RMS_RAW display) */
static uint32_t rms_raw(uint32_t ch) {
    uint64_t sum_sq = 0;
    for (int i = 0; i < RMS_SAMPLES; i++) {
        uint32_t s = adc2_read(ch);
        sum_sq += (uint64_t)s * s;
    }
    return isqrt((uint32_t)(sum_sq / RMS_SAMPLES));
}

static void print_sat_warning(int32_t mt_x100) {
    int32_t abs_mt = mt_x100 < 0 ? -mt_x100 : mt_x100;
    if (abs_mt > LINEAR_RANGE_MT_X100) uart_puts(" SAT!");
}

/* ─── PWM control ────────────────────────────────────────────────────────── */

/*
 * Apply drv_mode and pwm_duty to TIM1 CCR registers.
 *
 * DRV8874 IN/IN mode (PHEN=GND):
 *   Forward:  IN1=PWM, IN2=0
 *   Reverse:  IN1=0,   IN2=PWM
 *   Brake:    IN1=ARR, IN2=ARR  (both high → brake)
 *   Coast:    IN1=0,   IN2=0    (both low  → coast)
 */
static void pwm_apply(void) {
    switch (drv_mode) {
        case DRV_FORWARD:
            TIM1->CCR1 = pwm_duty;
            TIM1->CCR2 = 0;
            break;
        case DRV_REVERSE:
            TIM1->CCR1 = 0;
            TIM1->CCR2 = pwm_duty;
            break;
        case DRV_BRAKE:
            TIM1->CCR1 = PWM_ARR;
            TIM1->CCR2 = PWM_ARR;
            break;
        case DRV_COAST:
        default:
            TIM1->CCR1 = 0;
            TIM1->CCR2 = 0;
            break;
    }
}

static void pwm_print_state(void) {
    const char *mode_str;
    switch (drv_mode) {
        case DRV_FORWARD: mode_str = "FORWARD"; break;
        case DRV_REVERSE: mode_str = "REVERSE"; break;
        case DRV_BRAKE:   mode_str = "BRAKE";   break;
        case DRV_COAST:   mode_str = "COAST";   break;
        default:          mode_str = "UNKNOWN"; break;
    }
    /* duty% = pwm_duty * 100 / PWM_ARR — integer, good enough for display */
    uint32_t duty_pct = (pwm_duty * 100UL) / PWM_ARR;
    uart_puts("PWM: ");
    uart_puts(mode_str);
    uart_puts("  duty=");
    uart_putn(duty_pct);
    uart_puts("%  CCR=");
    uart_putn(pwm_duty);
    uart_puts("\r\n");
}

/* ─── DRV Status Reporting ──────────────────────────────────────────────── */
static void drv_print_status(void) {
    /* nSLEEP is on PB6 (output, read from ODR). Active low. */
    uint8_t nsleep_pin = (GPIOB->ODR >> 6) & 1U;
    /* nFAULT is on PB7 (input, read from IDR). Active low. */
    uint8_t nfault_pin = (GPIOB->IDR >> 7) & 1U;

    uart_puts("DRV: nSLEEP=");
    uart_putn(nsleep_pin);
    uart_puts(nsleep_pin ? "(AWAKE) " : "(SLEEP) ");
    
    uart_puts("nFAULT=");
    uart_putn(nfault_pin);
    uart_puts(nfault_pin ? "(OK)" : "(FAULT)");
    uart_puts("\r\n");
}

/* ─── Calibration ────────────────────────────────────────────────────────── */

static void calibrate(void) {
    uart_puts("CAL: sampling");
    int64_t acc0 = 0, acc1 = 0, acc_i = 0;
    for (int i = 0; i < CAL_SAMPLES; i++) {
        acc_i += (int64_t)adc2_read(1);   /* PA0 = ISENSE — must be zero current */
        acc0  += (int64_t)adc2_read(2);   /* PA1 = S1 */
        acc1  += (int64_t)adc2_read(3);   /* PA6 = S2 */
        if ((i & 63) == 63) uart_putc('.');
    }
    zero_isense   = (int32_t)(acc_i / CAL_SAMPLES);
    zero_count[0] = (int32_t)(acc0  / CAL_SAMPLES);
    zero_count[1] = (int32_t)(acc1  / CAL_SAMPLES);

    int32_t off0  = ((zero_count[0] - NOMINAL_MIDSCALE) * (int32_t)SENSITIVITY_NUM)
                    / (int32_t)SENSITIVITY_DEN;
    int32_t off1  = ((zero_count[1] - NOMINAL_MIDSCALE) * (int32_t)SENSITIVITY_NUM)
                    / (int32_t)SENSITIVITY_DEN;
    int32_t off_i = ((zero_isense - 2048) * 3300L) / 4096L;

    uart_puts("\r\nCAL DONE\r\n");
    uart_puts("S1     zero="); uart_putn((uint32_t)zero_count[0]);
    uart_puts("  offset_mT="); uart_put_fixed2(off0);  uart_puts("\r\n");
    uart_puts("S2     zero="); uart_putn((uint32_t)zero_count[1]);
    uart_puts("  offset_mT="); uart_put_fixed2(off1);  uart_puts("\r\n");
    uart_puts("ISENSE zero="); uart_putn((uint32_t)zero_isense);
    uart_puts("  offset_mA="); uart_put_fixed2(off_i / 10); uart_puts("\r\n");
    uart_puts("--------\r\n");
}

/* ─── Report ─────────────────────────────────────────────────────────────── */

static void do_full_report(void) {
    /* Power */
    uint32_t raw_isense = adc2_read(1);
    uint32_t raw_vbus   = adc2_read(4);
    int32_t  current_ma = raw_to_ma(raw_isense);
    uint32_t vbus_mv    = raw_to_vbus_mv(raw_vbus);

    /* Hall instantaneous */
    uint32_t raw1 = adc2_read(2);
    uint32_t raw2 = adc2_read(3);
    int32_t  mt1  = raw_to_mt_x100(raw1, 0);
    int32_t  mt2  = raw_to_mt_x100(raw2, 1);

    /* Hall RMS */
    uint32_t rms1_raw = rms_raw(2);
    uint32_t rms2_raw = rms_raw(3);
    uint32_t rms1_mt  = ac_rms_mt_x100(2, 0);
    uint32_t rms2_mt  = ac_rms_mt_x100(3, 1);

    /* Print power */
    uart_puts("=== POWER ===\r\n");
    uart_puts("VBUS=");
    uart_put_fixed2((int32_t)(vbus_mv / 10));   /* mV/10 as ×100 → prints X.XX V */
    uart_puts("V  VBUS_RAW="); uart_putn(raw_vbus); uart_puts("\r\n");

    uart_puts("ISENSE=");
    uart_put_fixed2(current_ma / 10);           /* mA/10 as ×100 → prints X.XX A */
    uart_puts("A  ISENSE_RAW="); uart_putn(raw_isense);
    {
        int32_t abs_ma = current_ma < 0 ? -current_ma : current_ma;
        if (abs_ma > 100) uart_puts("  *** >100mA CHECK ***");
    }
    uart_puts("\r\n");

    /* Print hall */
    uart_puts("=== INSTANT ===\r\n");
    uart_puts("S1(PA1): RAW="); uart_putn(raw1);
    uart_puts("  mT=");         uart_put_fixed2(mt1);
    print_sat_warning(mt1);     uart_puts("\r\n");

    uart_puts("S2(PA6): RAW="); uart_putn(raw2);
    uart_puts("  mT=");         uart_put_fixed2(mt2);
    print_sat_warning(mt2);     uart_puts("\r\n");

    uart_puts("=== RMS (");
    uart_putn(RMS_SAMPLES);
    uart_puts(" samples, AC, zeroed) ===\r\n");
    uart_puts("S1(PA1): RMS_RAW="); uart_putn(rms1_raw);
    uart_puts("  AC_RMS_mT=");      uart_put_fixed2((int32_t)rms1_mt);
    uart_puts("\r\n");
    uart_puts("S2(PA6): RMS_RAW="); uart_putn(rms2_raw);
    uart_puts("  AC_RMS_mT=");      uart_put_fixed2((int32_t)rms2_mt);
    uart_puts("\r\n");

    /* Print PWM and DRV state inline with report */
    pwm_print_state();
    drv_print_status();

    uart_puts("--------\r\n");
}

/* Stream all four channels — stops on any incoming byte */
static void do_stream(void) {
    uart_puts("STREAM START (any key to stop)\r\n");
    uart_puts("VBUS_RAW S1_RAW S2_RAW ISENSE_RAW\r\n");
    while (1) {
        if (rx_available()) {
            (void)rx_getc();
            break;
        }
        uint32_t vbus   = adc2_read(4);
        uint32_t s1     = adc2_read(2);
        uint32_t s2     = adc2_read(3);
        uint32_t isense = adc2_read(1);
        uart_putn(vbus);   uart_putc(' ');
        uart_putn(s1);     uart_putc(' ');
        uart_putn(s2);     uart_putc(' ');
        uart_putn(isense); uart_puts("\r\n");
    }
    uart_puts("STREAM STOP\r\n");
}

/* ─── Main ───────────────────────────────────────────────────────────────── */

int main(void) {

    /* 1. Clock — switch to 16 MHz HSI, no PLL */
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_HSI;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);

    /* 2. Peripheral clocks */
    RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOAEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
    RCC->AHB2ENR  |= RCC_AHB2ENR_ADC12EN;
    RCC->APB2ENR  |= RCC_APB2ENR_TIM1EN;

    /* nSLEEP on PB6 — drive high to enable DRV8874 */
    RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOBEN;
    GPIOB->MODER  &= ~(3U << (6*2));
    GPIOB->MODER  |=  (1U << (6*2));   /* output */
    GPIOB->BSRR    =  (1U << 6);       /* set high */

    /* nFAULT on PB7 — input with pull-up (DRV8874 nFAULT is active-low open-drain) */
    GPIOB->MODER  &= ~(3U << (7*2));   /* input mode (00) */
    GPIOB->PUPDR  &= ~(3U << (7*2));
    GPIOB->PUPDR  |=  (1U << (7*2));   /* pull-up */

    /* 3. PA2=TX, PA3=RX (USART2, AF7) */
    GPIOA->MODER &= ~((3U << (2*2)) | (3U << (3*2)));
    GPIOA->MODER |=  ((2U << (2*2)) | (2U << (3*2)));
    GPIOA->AFR[0] &= ~((0xFU << (2*4)) | (0xFU << (3*4)));
    GPIOA->AFR[0] |=  ((7U   << (2*4)) | (7U   << (3*4)));

    /* 4. Analog inputs: PA0(ISENSE), PA1(S1), PA6(S2), PA7(VBUS) */
    GPIOA->MODER |= (3U << (0*2));
    GPIOA->MODER |= (3U << (1*2));
    GPIOA->MODER |= (3U << (6*2));
    GPIOA->MODER |= (3U << (7*2));

    /* 5. PA8=TIM1_CH1 (AF6), PA9=TIM1_CH2 (AF6) */
    GPIOA->MODER &= ~((3U << (8*2)) | (3U << (9*2)));
    GPIOA->MODER |=  ((2U << (8*2)) | (2U << (9*2)));
    GPIOA->AFR[1] &= ~((0xFU << ((8-8)*4)) | (0xFU << ((9-8)*4)));
    GPIOA->AFR[1] |=  ((6U   << ((8-8)*4)) | (6U   << ((9-8)*4)));

    /* 6. USART2: 9600 baud at 16 MHz (BRR = 16000000 / 9600 = 1667) */
    USART2->BRR = 1667;
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
    USART2->CR1 |= USART_CR1_RXNEIE;          // enable RXNE interrupt
    NVIC_EnableIRQ(USART2_IRQn);              // enable in NVIC

    /* 7. ADC2 clock = SYSCLK */
    RCC->CCIPR |= RCC_CCIPR_ADC12SEL_1;

    /* 8. ADC2 voltage regulator enable + wait ≥20 µs */
    ADC2->CR = 0;
    ADC2->CR |= ADC_CR_ADVREGEN;
    for (volatile int i = 0; i < 10000; i++);

    /* 9. ADC2 self-calibration */
    ADC2->CR |= ADC_CR_ADCAL;
    while (ADC2->CR & ADC_CR_ADCAL);

    /* 10. ADC2 enable */
    ADC2->CR |= ADC_CR_ADEN;
    while (!(ADC2->ISR & ADC_ISR_ADRDY));

    /* 11. TIM1 — 16 kHz PWM, both channels, coast at boot
     *
     *  PSC=0, ARR=999: tick = 16MHz, period = 1000 ticks = 16kHz
     *  CH1 = PA8 = IN1, CH2 = PA9 = IN2
     *  OCM=110 (PWM mode 1): output high while CNT < CCR
     *  TIM_BDTR_MOE is mandatory for TIM1 (advanced timer) — without it
     *  outputs stay low regardless of CCR value.
     */
    TIM1->PSC   = 0;
    TIM1->ARR   = PWM_ARR;
    TIM1->CCR1  = 0;
    TIM1->CCR2  = 0;
    TIM1->CCMR1 = (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE |
                  (6U << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
    TIM1->CCER  = TIM_CCER_CC1E | TIM_CCER_CC2E;
    TIM1->BDTR |= TIM_BDTR_MOE;
    TIM1->CR1  |= TIM_CR1_ARPE | TIM_CR1_CEN;

    /* 12. Boot banner */
    uart_puts("BOOT OK\r\n");
    uart_puts("Hall: DRV5055A1 60mV/mT +/-22mT | Current: INA240A1 1V/A | VBUS: /11\r\n");
    uart_puts("PWM:  TIM1 16kHz | DRV8874 IN/IN mode | coast at boot\r\n");
    uart_puts("Cmds: R=report  A=auto  S=stream  Z=recal  D=drv status\r\n");
    uart_puts("      F=forward  V=reverse  B=brake  C=coast  +=duty+10%  -=duty-10%  P=pwm  N=nSleep toggle\r\n");
    uart_puts("--------\r\n");

    /* 13. Boot calibration — coils must be off, no field sources */
    uart_puts("Boot calibration (zero current, zero field)...\r\n");
    calibrate();

    uint8_t auto_mode = 0;

    /* ── Main loop ──────────────────────────────────────────────────────── */
    while (1) {

        if (rx_available()) {
            char c = rx_getc();

            switch (c) {

                /* ── Sensor / reporting commands ── */
                case 'R':
                    do_full_report();
                    break;

                case 'A':
                    auto_mode = !auto_mode;
                    uart_puts(auto_mode ? "AUTO ON\r\n" : "AUTO OFF\r\n");
                    break;

                case 'S':
                    do_stream();
                    break;

                case 'Z':
                    /* Coast first so ISENSE zero is meaningful */
                    drv_mode = DRV_COAST;
                    pwm_apply();
                    uart_puts("Coasted. Ensure zero field before cal...\r\n");
                    calibrate();
                    break;

                /* ── PWM / drive commands ── */
                case 'F':
                    drv_mode = DRV_FORWARD;
                    pwm_apply();
                    uart_puts("FORWARD\r\n");
                    pwm_print_state();
                    break;

                case 'V':
                    drv_mode = DRV_REVERSE;
                    pwm_apply();
                    uart_puts("REVERSE\r\n");
                    pwm_print_state();
                    break;

                case 'B':
                    drv_mode = DRV_BRAKE;
                    pwm_apply();
                    uart_puts("BRAKE\r\n");
                    pwm_print_state();
                    break;

                case 'C':
                    drv_mode = DRV_COAST;
                    pwm_apply();
                    uart_puts("COAST\r\n");
                    pwm_print_state();
                    break;

                case '+':
                    if (pwm_duty + PWM_DUTY_STEP <= PWM_MAX)
                        pwm_duty += PWM_DUTY_STEP;
                    else
                        pwm_duty = PWM_MAX;
                    pwm_apply();
                    pwm_print_state();
                    break;

                case '-':
                    if (pwm_duty >= PWM_DUTY_STEP)
                        pwm_duty -= PWM_DUTY_STEP;
                    else
                        pwm_duty = PWM_MIN;
                    pwm_apply();
                    pwm_print_state();
                    break;

                case 'P':
                    pwm_print_state();
                    drv_print_status();
                    break;

                case 'D':
                    drv_print_status();
                    break;
                
                case 'N':
                    GPIOB->BSRR = (1U << (6 + 16));  /* pull low  */
                    for (volatile int i = 0; i < 10000; i++);  /* brief delay */
                    GPIOB->BSRR = (1U << 6);          /* pull high */
                    uart_puts("nSLEEP toggled (clears latched faults)\r\n");
                    break;

                default:
                    /* Silently ignore unknown bytes */
                    break;
            }
        }

        if (auto_mode) {
            do_full_report();
            /* ~200 ms delay at 16 MHz: 3,200,000 cycles ≈ 200 ms */
            for (volatile uint32_t d = 0; d < 3200000UL; d++);
        }
    }
}