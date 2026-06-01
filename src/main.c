/*
 * main.c - Dual DRV5055A1EDBZRQ1 Hall Sensor RMS Firmware
 * STM32G431CBU6 | Raw registers only (no HAL) | 16MHz HSI
 *
 * Sensors: Texas Instruments DRV5055A1EDBZRQ1
 *   PA1 = ADC2_IN2  (Sensor 1)
 *   PA6 = ADC2_IN3  (Sensor 2)
 *
 * UART: PA2=TX, PA3=RX, 9600 baud → Raspberry Pi 5
 *
 * ── DRV5055A1 specs at VCC = 3.3V ───────────────────────────────────────────
 *  Sensitivity:       60 mV/mT (typ), range 57–63 mV/mT
 *  Quiescent voltage: VCC/2 = 1.65V (1.59–1.71V tolerance)
 *  Linear range:      ±22 mT
 *  Bandwidth:         20 kHz
 *
 * ── Ratiometric note ────────────────────────────────────────────────────────
 *  The DRV5055 output and quiescent point both scale with VCC.
 *  Since the STM32 ADC reference is also VCC, the ratio cancels supply
 *  variation. Nominal midscale = 2048 counts. After boot calibration the
 *  per-sensor zero_count replaces 2048 for all conversions.
 *
 * ── Zero-offset calibration ─────────────────────────────────────────────────
 *  At startup (before any field is applied) the firmware samples each sensor
 *  for CAL_SAMPLES readings and averages them. The result is stored as
 *  zero_count[0] and zero_count[1]. All subsequent mT conversions subtract
 *  this measured zero instead of the nominal 2048, correcting:
 *    - Sensor quiescent output tolerance (±60 mV = ~74 LSB)
 *    - Static ambient field (earth field ~0.03–0.06 mT)
 *    - Any PCB-level DC magnetic bias
 *
 *  Send 'Z' at any time to re-run calibration (e.g. after moving the board).
 *  The calibration result is reported over UART for logging.
 *
 *  IMPORTANT: sensors must be away from the field source during calibration.
 *
 * ── AC RMS strategy ─────────────────────────────────────────────────────────
 *  AC RMS = RMS of (raw - zero_count), removing the DC bias before squaring.
 *  This correctly measures the AC field magnitude regardless of offset.
 *
 * ── Commands ────────────────────────────────────────────────────────────────
 *  'R' = single full report (instant snapshot + RMS window)
 *  'A' = toggle auto-report (~200ms interval)
 *  'S' = stream raw ADC pairs until any keypress
 *  'Z' = re-run zero calibration
 */

#include "stm32g4xx.h"

/* ─── Sensitivity constants (ratiometric, VCC=3.3V) ─────────────────────────
 *
 * delta_mT × 100 = (raw - zero_count) * 330000 / 245760
 *
 * Overflow check: max |raw - zero| = ~2048
 *   2048 * 330000 = 675,840,000 → fits in int32_t ✓
 */
#define SENSITIVITY_NUM       330000    /* 3300 mV * 100                        */
#define SENSITIVITY_DEN       245760    /* 4096 counts * 60 mV/mT              */
#define NOMINAL_MIDSCALE      2048      /* fallback if cal fails                */
#define LINEAR_RANGE_MT_X100  2200      /* ±22.00 mT saturation threshold       */

/* Number of samples averaged for zero calibration (~512 at ~6us each = ~3ms) */
#define CAL_SAMPLES           512

/* RMS window depth */
#define RMS_SAMPLES           512

/* ─── Per-sensor zero offsets (set by calibrate(), used everywhere) ──────── */
static int32_t zero_count[2] = { NOMINAL_MIDSCALE, NOMINAL_MIDSCALE };

/* ─── UART ──────────────────────────────────────────────────────────────────── */

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

/* Print signed value as "int.frac" with 2 decimal places (val is ×100) */
void uart_put_fixed2(int32_t val_x100) {
    if (val_x100 < 0) { uart_putc('-'); val_x100 = -val_x100; }
    uart_putn((uint32_t)(val_x100 / 100));
    uart_putc('.');
    uint32_t frac = (uint32_t)(val_x100 % 100);
    if (frac < 10) uart_putc('0');
    uart_putn(frac);
}

/* ─── ADC ───────────────────────────────────────────────────────────────────── */

/* Read one sample from ADC2 channel ch (2=PA1, 3=PA6). */
static inline uint32_t adc2_read(uint32_t ch) {
    ADC2->SQR1  = (ch << 6);
    ADC2->SMPR1 = (7U << (ch * 3));
    ADC2->CFGR  = 0;
    ADC2->CR   |= ADC_CR_ADSTART;
    while (!(ADC2->ISR & ADC_ISR_EOC));
    return ADC2->DR;
}

/* ─── Zero calibration ──────────────────────────────────────────────────────
 *
 * Averages CAL_SAMPLES readings for each sensor with no field applied.
 * Stores the result in zero_count[]. Uses int64 accumulator to avoid
 * overflow: 4095 * 512 = 2,096,640 — fine in int32, but int64 is safer
 * if CAL_SAMPLES is ever increased.
 */
static void calibrate(void) {
    uart_puts("CAL: sampling");

    int64_t acc0 = 0, acc1 = 0;
    for (int i = 0; i < CAL_SAMPLES; i++) {
        acc0 += (int64_t)adc2_read(2);   /* PA1 = ch2 = sensor 1 */
        acc1 += (int64_t)adc2_read(3);   /* PA6 = ch3 = sensor 2 */
        /* Print a dot every 64 samples so the user sees activity */
        if ((i & 63) == 63) uart_putc('.');
    }

    zero_count[0] = (int32_t)(acc0 / CAL_SAMPLES);
    zero_count[1] = (int32_t)(acc1 / CAL_SAMPLES);

    uart_puts("\r\nCAL DONE\r\n");
    uart_puts("S1 zero_count=");
    uart_putn((uint32_t)zero_count[0]);
    uart_puts("  offset_mT=");
    /* Show what offset was removed — (zero - nominal) in mT */
    int32_t off0 = ((zero_count[0] - NOMINAL_MIDSCALE) * (int32_t)SENSITIVITY_NUM)
                   / (int32_t)SENSITIVITY_DEN;
    int32_t off1 = ((zero_count[1] - NOMINAL_MIDSCALE) * (int32_t)SENSITIVITY_NUM)
                   / (int32_t)SENSITIVITY_DEN;
    uart_put_fixed2(off0);
    uart_puts("\r\n");
    uart_puts("S2 zero_count=");
    uart_putn((uint32_t)zero_count[1]);
    uart_puts("  offset_mT=");
    uart_put_fixed2(off1);
    uart_puts("\r\n--------\r\n");
}

/* ─── Math ──────────────────────────────────────────────────────────────────── */

static uint32_t isqrt(uint32_t n) {
    if (n == 0) return 0;
    uint32_t x = n, y = (x + 1) / 2;
    while (y < x) { x = y; y = (x + n / x) / 2; }
    return x;
}

/*
 * raw → mT × 100, zeroed against calibrated offset.
 * sensor: 0 or 1 (indexes zero_count[]).
 */
static int32_t raw_to_mt_x100(uint32_t raw, int sensor) {
    int32_t delta = (int32_t)raw - zero_count[sensor];
    return (delta * (int32_t)SENSITIVITY_NUM) / (int32_t)SENSITIVITY_DEN;
}

/* raw → millivolts (diagnostic, assumes VCC=3.3V) */
static uint32_t raw_to_mv(uint32_t raw) {
    return (raw * 3300UL) / 4096UL;
}

/*
 * AC RMS in mT × 100 for a given channel, zeroed against calibrated offset.
 * sensor: 0 or 1.
 *
 * Overflow: max |delta| after cal still ≤ ~2048 in practice (±22mT range),
 * delta^2 ≤ 4,194,304, × 512 = 2,147,483,648 → fits in uint64_t easily ✓
 */
static uint32_t ac_rms_mt_x100(uint32_t ch, int sensor) {
    uint64_t sum_sq = 0;
    for (int i = 0; i < RMS_SAMPLES; i++) {
        int32_t delta = (int32_t)adc2_read(ch) - zero_count[sensor];
        sum_sq += (uint64_t)((int64_t)delta * delta);
    }
    uint32_t rms_counts = isqrt((uint32_t)(sum_sq / RMS_SAMPLES));
    return (rms_counts * (uint32_t)SENSITIVITY_NUM) / (uint32_t)SENSITIVITY_DEN;
}

/* RMS of raw counts (diagnostic) */
static uint32_t rms_raw(uint32_t ch) {
    uint64_t sum_sq = 0;
    for (int i = 0; i < RMS_SAMPLES; i++) {
        uint32_t s = adc2_read(ch);
        sum_sq += (uint64_t)s * s;
    }
    return isqrt((uint32_t)(sum_sq / RMS_SAMPLES));
}

/* ─── Saturation check ──────────────────────────────────────────────────────── */

static void print_sat_warning(int32_t mt_x100) {
    int32_t abs_mt = mt_x100 < 0 ? -mt_x100 : mt_x100;
    if (abs_mt > LINEAR_RANGE_MT_X100) uart_puts(" SAT!");
}

/* ─── Report ────────────────────────────────────────────────────────────────── */

void do_full_report(void) {
    /* ── Instantaneous snapshot ───────────────────────── */
    uint32_t raw1 = adc2_read(2);
    uint32_t raw2 = adc2_read(3);

    uint32_t mv1  = raw_to_mv(raw1);
    uint32_t mv2  = raw_to_mv(raw2);

    int32_t  mt1  = raw_to_mt_x100(raw1, 0);
    int32_t  mt2  = raw_to_mt_x100(raw2, 1);

    /* ── RMS window ───────────────────────────────────── */
    uint32_t rms1_raw = rms_raw(2);
    uint32_t rms2_raw = rms_raw(3);
    uint32_t rms1_mv  = raw_to_mv(rms1_raw);
    uint32_t rms2_mv  = raw_to_mv(rms2_raw);
    uint32_t rms1_mt  = ac_rms_mt_x100(2, 0);
    uint32_t rms2_mt  = ac_rms_mt_x100(3, 1);

    /* ── Print ────────────────────────────────────────── */
    uart_puts("=== INSTANT ===\r\n");

    uart_puts("S1(PA1): RAW=");  uart_putn(raw1);
    uart_puts("  mV=");          uart_putn(mv1);
    uart_puts("  mT=");          uart_put_fixed2(mt1);
    print_sat_warning(mt1);      uart_puts("\r\n");

    uart_puts("S2(PA6): RAW=");  uart_putn(raw2);
    uart_puts("  mV=");          uart_putn(mv2);
    uart_puts("  mT=");          uart_put_fixed2(mt2);
    print_sat_warning(mt2);      uart_puts("\r\n");

    uart_puts("=== RMS (");
    uart_putn(RMS_SAMPLES);
    uart_puts(" samples, AC, zeroed) ===\r\n");

    uart_puts("S1(PA1): RMS_RAW="); uart_putn(rms1_raw);
    uart_puts("  RMS_mV=");         uart_putn(rms1_mv);
    uart_puts("  AC_RMS_mT=");      uart_put_fixed2((int32_t)rms1_mt);
    uart_puts("\r\n");

    uart_puts("S2(PA6): RMS_RAW="); uart_putn(rms2_raw);
    uart_puts("  RMS_mV=");         uart_putn(rms2_mv);
    uart_puts("  AC_RMS_mT=");      uart_put_fixed2((int32_t)rms2_mt);
    uart_puts("\r\n");

    uart_puts("--------\r\n");
}

/* Stream raw pairs — stops on any incoming byte */
void do_stream(void) {
    uart_puts("STREAM START (any key to stop)\r\n");
    while (1) {
        if (USART2->ISR & USART_ISR_RXNE) {
            (void)USART2->RDR;
            break;
        }
        uint32_t s1 = adc2_read(2);
        uint32_t s2 = adc2_read(3);
        uart_puts("S1="); uart_putn(s1);
        uart_puts(" S2="); uart_putn(s2);
        uart_puts("\r\n");
    }
    uart_puts("STREAM STOP\r\n");
}

/* ─── Main ──────────────────────────────────────────────────────────────────── */

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

    /* 3. PA2 = USART2 TX (AF7), PA3 = USART2 RX (AF7) */
    GPIOA->MODER &= ~((3U << (2*2)) | (3U << (2*3)));
    GPIOA->MODER |=  ((2U << (2*2)) | (2U << (2*3)));
    GPIOA->AFR[0] &= ~((0xFU << (2*4)) | (0xFU << (3*4)));
    GPIOA->AFR[0] |=  ((7U   << (2*4)) | (7U   << (3*4)));

    /* 4. PA1 = analog (ADC2_IN2), PA6 = analog (ADC2_IN3) */
    GPIOA->MODER |= (3U << (1*2));
    GPIOA->MODER |= (3U << (6*2));

    /* 5. USART2: 9600 baud at 16MHz */
    USART2->BRR = 1667;
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;

    /* 6. ADC2 clock = SYSCLK */
    RCC->CCIPR |= RCC_CCIPR_ADC12SEL_1;

    /* 7. ADC voltage regulator */
    ADC2->CR = 0;
    ADC2->CR |= ADC_CR_ADVREGEN;
    for (volatile int i = 0; i < 10000; i++);

    /* 8. Calibrate ADC2 */
    ADC2->CR |= ADC_CR_ADCAL;
    while (ADC2->CR & ADC_CR_ADCAL);

    /* 9. Enable ADC2 */
    ADC2->CR |= ADC_CR_ADEN;
    while (!(ADC2->ISR & ADC_ISR_ADRDY));

    uart_puts("BOOT OK\r\n");
    uart_puts("DRV5055A1 dual sensor | 60mV/mT | range +/-22mT\r\n");
    uart_puts("R=report  A=auto  S=stream  Z=recalibrate\r\n");
    uart_puts("--------\r\n");

    /* Run zero calibration at boot — ensure no field present */
    uart_puts("Boot calibration (remove field sources now)...\r\n");
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