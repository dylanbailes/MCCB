/*
 * main.c - Dual DRV5055A1EDBZRQ1 Hall Sensor RMS Firmware
 * STM32G431CBU6 | Raw registers only (no HAL) | 16MHz HSI
 *
 * ── ADC2 channel map ────────────────────────────────────────────────────────
 *  PA0 = ADC2_IN1  ISENSE_OUT  (INA240A1, 50mΩ shunt, gain 20 → 1V/A)
 *  PA1 = ADC2_IN2  Hall S1     (DRV5055A1, 60mV/mT)
 *  PA6 = ADC2_IN3  Hall S2     (DRV5055A1, 60mV/mT)
 *  PA7 = ADC2_IN4  V_BUS_MON  (100k/10k divider → ÷11, reads up to ~36V)
 *
 * ── UART ────────────────────────────────────────────────────────────────────
 *  PA2 = USART2 TX (AF7) → Pi GPIO15
 *  PA3 = USART2 RX (AF7) → Pi GPIO14
 *  9600 baud, 16MHz HSI
 *
 * ── DRV5055A1 Hall sensors ──────────────────────────────────────────────────
 *  Sensitivity: 60 mV/mT @ 3.3V, ratiometric (midscale = VCC/2)
 *  Range: ±22 mT. Boot calibration removes per-sensor DC offset.
 *  AC RMS computed over RMS_SAMPLES for 0–250Hz field measurement.
 *
 * ── INA240A1 current sense ──────────────────────────────────────────────────
 *  Gain: 20 V/V. Shunt: 50mΩ (WSL2512R0500FEA)
 *  Output: 1.0 V/A  (20 × 0.05 = 1.0)
 *  Resolution: 3.3V / 4096 = 0.806 mV/count = 0.806 mA/count
 *  Range: 0–3.3A before ADC clips
 *  VREF pin tied to GND → output = 0V at 0A
 *  Formula: I_mA = raw * 3300 / 4096   (since 1V/A → 1mV/mA)
 *
 * ── V_BUS_MON voltage divider ───────────────────────────────────────────────
 *  R5=100kΩ (to +24V_BUS), R4=10kΩ (to GND) → ratio = 10/110 = 1/11
 *  V_BUS = V_ADC × 11
 *  At 24V: V_ADC = 2.18V → raw ≈ 2703 counts
 *  Formula: V_BUS_mV = raw * 3300 * 11 / 4096 = raw * 36300 / 4096
 *  Overflow check: 4095 * 36300 = 148,648,500 → fits in uint32_t ✓
 *
 * ── Commands ────────────────────────────────────────────────────────────────
 *  'R' = single full report (hall sensors + current + bus voltage)
 *  'A' = toggle auto-report (~200ms interval)
 *  'S' = stream all four raw ADC channels until any keypress
 *  'Z' = re-run hall sensor zero calibration
 */

#include "stm32g4xx.h"

/* ─── Hall sensor constants ──────────────────────────────────────────────── */
#define SENSITIVITY_NUM       330000    /* 3300mV × 100 for ×100 fixed-point   */
#define SENSITIVITY_DEN       245760    /* 4096 counts × 60 mV/mT              */
#define NOMINAL_MIDSCALE      2048
#define LINEAR_RANGE_MT_X100  2200      /* ±22.00 mT                           */
#define CAL_SAMPLES           512
#define RMS_SAMPLES           512

/* ─── Per-sensor zero offsets ────────────────────────────────────────────── */
static int32_t zero_count[2] = { NOMINAL_MIDSCALE, NOMINAL_MIDSCALE };

/* ─── UART ───────────────────────────────────────────────────────────────── */

void uart_putc(char c) {
    while (!(USART2->ISR & USART_ISR_TXE));
    USART2->TDR = c;
}

void uart_puts(const char *s) {
    while (*s) uart_putc(*s++);
}

void uart_putn(uint32_t n) {
    char buf[12];
    int i = 0;
    if (n == 0) { uart_putc('0'); return; }
    while (n > 0) { buf[i++] = '0' + (n % 10); n /= 10; }
    while (i--) uart_putc(buf[i]);
}

/* Print signed ×100 fixed-point as "X.XX" */
void uart_put_fixed2(int32_t val_x100) {
    if (val_x100 < 0) { uart_putc('-'); val_x100 = -val_x100; }
    uart_putn((uint32_t)(val_x100 / 100));
    uart_putc('.');
    uint32_t frac = (uint32_t)(val_x100 % 100);
    if (frac < 10) uart_putc('0');
    uart_putn(frac);
}

/* ─── ADC ────────────────────────────────────────────────────────────────── */

/*
 * Single conversion on ADC2 channel ch.
 * Channels used: 1=PA0, 2=PA1, 3=PA6, 4=PA7
 *
 * SMPR1 covers channels 1–9, 3 bits each:
 *   ch1 → bits [5:3], ch2 → bits [8:6], ch3 → bits [11:9], ch4 → bits [14:12]
 *   shift = ch * 3
 */
static inline uint32_t adc2_read(uint32_t ch) {
    ADC2->SQR1  = (ch << 6);
    ADC2->SMPR1 = (7U << (ch * 3));   /* max sampling time */
    ADC2->CFGR  = 0;
    ADC2->CR   |= ADC_CR_ADSTART;
    while (!(ADC2->ISR & ADC_ISR_EOC));
    return ADC2->DR;
}

/* ─── Conversions ────────────────────────────────────────────────────────── */

static uint32_t isqrt(uint32_t n) {
    if (n == 0) return 0;
    uint32_t x = n, y = (x + 1) / 2;
    while (y < x) { x = y; y = (x + n / x) / 2; }
    return x;
}

/* raw → mT × 100, zeroed per-sensor */
static int32_t raw_to_mt_x100(uint32_t raw, int sensor) {
    int32_t delta = (int32_t)raw - zero_count[sensor];
    return (delta * (int32_t)SENSITIVITY_NUM) / (int32_t)SENSITIVITY_DEN;
}

/* raw → millivolts (3.3V reference) */
static uint32_t raw_to_mv(uint32_t raw) {
    return (raw * 3300UL) / 4096UL;
}

/*
 * raw → current in mA
 * INA240A1: gain=20, shunt=50mΩ → 1.0 V/A → 1 mV/mA
 * I_mA = V_mV / 1.0 = raw * 3300 / 4096
 */
static uint32_t raw_to_ma(uint32_t raw) {
    return (raw * 3300UL) / 4096UL;
}

/*
 * raw → bus voltage in mV
 * Divider ratio 1/11: V_BUS_mV = raw * 3300 * 11 / 4096
 */
static uint32_t raw_to_vbus_mv(uint32_t raw) {
    return (raw * 36300UL) / 4096UL;
}

/* AC RMS in mT × 100, zeroed per-sensor */
static uint32_t ac_rms_mt_x100(uint32_t ch, int sensor) {
    uint64_t sum_sq = 0;
    for (int i = 0; i < RMS_SAMPLES; i++) {
        int32_t delta = (int32_t)adc2_read(ch) - zero_count[sensor];
        sum_sq += (uint64_t)((int64_t)delta * delta);
    }
    uint32_t rms_counts = isqrt((uint32_t)(sum_sq / RMS_SAMPLES));
    return (rms_counts * (uint32_t)SENSITIVITY_NUM) / (uint32_t)SENSITIVITY_DEN;
}

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

/* ─── Calibration ────────────────────────────────────────────────────────── */

static void calibrate(void) {
    uart_puts("CAL: sampling");
    int64_t acc0 = 0, acc1 = 0;
    for (int i = 0; i < CAL_SAMPLES; i++) {
        acc0 += (int64_t)adc2_read(2);   /* PA1 = S1 */
        acc1 += (int64_t)adc2_read(3);   /* PA6 = S2 */
        if ((i & 63) == 63) uart_putc('.');
    }
    zero_count[0] = (int32_t)(acc0 / CAL_SAMPLES);
    zero_count[1] = (int32_t)(acc1 / CAL_SAMPLES);

    int32_t off0 = ((zero_count[0] - NOMINAL_MIDSCALE) * (int32_t)SENSITIVITY_NUM)
                   / (int32_t)SENSITIVITY_DEN;
    int32_t off1 = ((zero_count[1] - NOMINAL_MIDSCALE) * (int32_t)SENSITIVITY_NUM)
                   / (int32_t)SENSITIVITY_DEN;

    uart_puts("\r\nCAL DONE\r\n");
    uart_puts("S1 zero_count="); uart_putn((uint32_t)zero_count[0]);
    uart_puts("  offset_mT=");   uart_put_fixed2(off0); uart_puts("\r\n");
    uart_puts("S2 zero_count="); uart_putn((uint32_t)zero_count[1]);
    uart_puts("  offset_mT=");   uart_put_fixed2(off1); uart_puts("\r\n");
    uart_puts("--------\r\n");
}

/* ─── Report ─────────────────────────────────────────────────────────────── */

void do_full_report(void) {
    /* ── Power monitoring ─────────────────────────── */
    uint32_t raw_isense = adc2_read(1);   /* PA0 = ADC2_IN1 */
    uint32_t raw_vbus   = adc2_read(4);   /* PA7 = ADC2_IN4 */

    uint32_t current_ma = raw_to_ma(raw_isense);
    uint32_t vbus_mv    = raw_to_vbus_mv(raw_vbus);

    /* ── Hall instantaneous ───────────────────────── */
    uint32_t raw1 = adc2_read(2);
    uint32_t raw2 = adc2_read(3);

    int32_t mt1 = raw_to_mt_x100(raw1, 0);
    int32_t mt2 = raw_to_mt_x100(raw2, 1);

    /* ── Hall RMS ─────────────────────────────────── */
    uint32_t rms1_raw = rms_raw(2);
    uint32_t rms2_raw = rms_raw(3);
    uint32_t rms1_mt  = ac_rms_mt_x100(2, 0);
    uint32_t rms2_mt  = ac_rms_mt_x100(3, 1);

    /* ── Print power ──────────────────────────────── */
    uart_puts("=== POWER ===\r\n");

    uart_puts("VBUS=");
    uart_put_fixed2((int32_t)(vbus_mv / 10));   /* mV/10 → ×100 for X.XX V */
    uart_puts("V  ");
    uart_puts("VBUS_RAW="); uart_putn(raw_vbus);
    uart_puts("\r\n");

    uart_puts("ISENSE=");
    uart_put_fixed2((int32_t)(current_ma / 10)); /* mA/10 → ×100 for X.XX A */
    uart_puts("A  ");
    uart_puts("ISENSE_RAW="); uart_putn(raw_isense);
    /* Flag unexpectedly high current at zero PWM */
    if (current_ma > 100) uart_puts("  *** >100mA CHECK ***");
    uart_puts("\r\n");

    /* ── Print hall ───────────────────────────────── */
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

    uart_puts("--------\r\n");
}

/* Stream all four channels — stops on any incoming byte */
void do_stream(void) {
    uart_puts("STREAM START (any key to stop)\r\n");
    uart_puts("VBUS_RAW S1_RAW S2_RAW ISENSE_RAW\r\n");
    while (1) {
        if (USART2->ISR & USART_ISR_RXNE) {
            (void)USART2->RDR;
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
    /* 1. Clock */
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_HSI;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);

    /* 2. Peripheral clocks */
    RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOAEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
    RCC->AHB2ENR  |= RCC_AHB2ENR_ADC12EN;

    /* 3. PA2=TX, PA3=RX (USART2, AF7) */
    GPIOA->MODER &= ~((3U << (2*2)) | (3U << (2*3)));
    GPIOA->MODER |=  ((2U << (2*2)) | (2U << (2*3)));
    GPIOA->AFR[0] &= ~((0xFU << (2*4)) | (0xFU << (3*4)));
    GPIOA->AFR[0] |=  ((7U   << (2*4)) | (7U   << (3*4)));

    /* 4. Analog inputs: PA0, PA1, PA6, PA7 */
    GPIOA->MODER |= (3U << (0*2));   /* PA0 = ISENSE  */
    GPIOA->MODER |= (3U << (1*2));   /* PA1 = Hall S1 */
    GPIOA->MODER |= (3U << (6*2));   /* PA6 = Hall S2 */
    GPIOA->MODER |= (3U << (7*2));   /* PA7 = V_BUS   */

    /* 5. USART2: 9600 baud at 16MHz */
    USART2->BRR = 1667;
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;

    /* 6. ADC2 clock = SYSCLK */
    RCC->CCIPR |= RCC_CCIPR_ADC12SEL_1;

    /* 7. ADC2 voltage regulator */
    ADC2->CR = 0;
    ADC2->CR |= ADC_CR_ADVREGEN;
    for (volatile int i = 0; i < 10000; i++);

    /* 8. ADC2 calibration */
    ADC2->CR |= ADC_CR_ADCAL;
    while (ADC2->CR & ADC_CR_ADCAL);

    /* 9. ADC2 enable */
    ADC2->CR |= ADC_CR_ADEN;
    while (!(ADC2->ISR & ADC_ISR_ADRDY));

    uart_puts("BOOT OK\r\n");
    uart_puts("Hall: DRV5055A1 60mV/mT +/-22mT | Current: INA240A1 1V/A | VBUS: /11 divider\r\n");
    uart_puts("R=report  A=auto  S=stream  Z=recalibrate\r\n");
    uart_puts("--------\r\n");

    uart_puts("Boot calibration (no field sources)...\r\n");
    calibrate();

    uint8_t auto_mode = 0;

    while (1) {
        if (USART2->ISR & USART_ISR_RXNE) {
            char c = (char)USART2->RDR;
            if      (c == 'R') { do_full_report(); }
            else if (c == 'A') { auto_mode = !auto_mode;
                                 uart_puts(auto_mode ? "AUTO ON\r\n" : "AUTO OFF\r\n"); }
            else if (c == 'S') { do_stream(); }
            else if (c == 'Z') { calibrate(); }
        }

        if (auto_mode) {
            do_full_report();
            for (volatile uint32_t d = 0; d < 3200000UL; d++);
        }
    }
}