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
 *  Quiescent voltage: VCC/2 = 1.65V (1.59–1.71V)
 *  Linear range:      ±22 mT
 *  Bandwidth:         20 kHz
 *  Temp compensation: +0.12%/°C (A-series compensates for magnet drift)
 *
 * ── Ratiometric note ────────────────────────────────────────────────────────
 *  The DRV5055 output and quiescent point both scale with VCC.
 *  Since the STM32 ADC reference is also VCC, the ratio (ADC count / 4096)
 *  already cancels supply variation. We therefore work in "counts relative
 *  to midscale" and apply a fixed fractional sensitivity:
 *
 *    S_frac = 60 mV/mT / 3300 mV = 0.018182 counts_fraction/mT
 *    => 1 LSB = VCC/4096, midscale = 2048 counts
 *    => delta_mT = (raw - 2048) / (4096 * S_frac)
 *               = (raw - 2048) / (4096 * 60/3300)
 *               = (raw - 2048) * 3300 / (4096 * 60)
 *
 *  All integer math, no floats. Result in mT × 100 for 2 decimal places.
 *
 * ── AC RMS strategy ─────────────────────────────────────────────────────────
 *  Sensors read AC magnetic fields (0–250 Hz).
 *  Output is DC-biased AC: idle at VCC/2, swings above/below for ±B.
 *  AC RMS = RMS of (sample - midscale), which is the field AC component.
 *  DC offset (midscale) cancels automatically in the subtraction.
 *
 *  RMS_SAMPLES = 512 samples per channel.
 *  At ~6 µs per dual-channel pair → ~3 ms window.
 *  Nyquist for 250 Hz = 500 Hz minimum; actual rate ~83 kHz. ✓
 *
 * ── Commands ────────────────────────────────────────────────────────────────
 *  'R' = single full report (instant snapshot + RMS window)
 *  'A' = toggle auto-report (~200ms interval)
 *  'S' = stream raw ADC pairs until any keypress
 */

#include "stm32g4xx.h"

/* ─── DRV5055A1 calibration (ratiometric, VCC=3.3V) ─────────────────────────
 *
 * SENSITIVITY_NUM / SENSITIVITY_DEN = mT_per_count * 100  (for 2dp output)
 *
 * delta_mT × 100 = (raw - MIDSCALE) * 3300 * 100 / (4096 * 60)
 *                = (raw - MIDSCALE) * 330000 / 245760
 *
 * To keep integer math safe with 12-bit ADC:
 *   max (raw - MIDSCALE) = ±2048
 *   2048 * 330000 = 675,840,000  → fits in int32_t (max ~2.1 billion) ✓
 */
#define MIDSCALE              2048      /* ADC count at B=0 (VCC/2 ratiometric) */
#define SENSITIVITY_NUM       330000    /* 3300 mV * 100 (for ×100 fixed-point) */
#define SENSITIVITY_DEN       245760    /* 4096 counts * 60 mV/mT              */

/* Clamp raw reading outside ±22mT linear range to flag saturation */
#define LINEAR_RANGE_MT_X100  2200      /* ±22.00 mT as ×100 value             */

/* RMS window depth — 512 samples per channel */
#define RMS_SAMPLES           512

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

void uart_putsi(int32_t n) {
    if (n < 0) { uart_putc('-'); n = -n; }
    uart_putn((uint32_t)n);
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
    ADC2->SMPR1 = (7U << (ch * 3));   /* max sampling time */
    ADC2->CFGR  = 0;                  /* single, software trigger */
    ADC2->CR   |= ADC_CR_ADSTART;
    while (!(ADC2->ISR & ADC_ISR_EOC));
    return ADC2->DR;
}

/* ─── Math ──────────────────────────────────────────────────────────────────── */

/* Integer square root (Newton's method) */
static uint32_t isqrt(uint32_t n) {
    if (n == 0) return 0;
    uint32_t x = n, y = (x + 1) / 2;
    while (y < x) { x = y; y = (x + n / x) / 2; }
    return x;
}

/*
 * Convert raw ADC count → mT × 100  (signed)
 *
 *   delta_mT×100 = (raw - MIDSCALE) * SENSITIVITY_NUM / SENSITIVITY_DEN
 *
 * Uses ratiometric cancellation: midscale = 2048 regardless of actual VCC.
 */
static int32_t raw_to_mt_x100(uint32_t raw) {
    int32_t delta = (int32_t)raw - MIDSCALE;
    return (delta * (int32_t)SENSITIVITY_NUM) / (int32_t)SENSITIVITY_DEN;
}

/*
 * raw → millivolts (for diagnostic logging only)
 * VCC assumed 3300 mV; not ratiometrically corrected (informational).
 */
static uint32_t raw_to_mv(uint32_t raw) {
    return (raw * 3300UL) / 4096UL;
}

/*
 * AC RMS in mT × 100 for a given channel.
 *
 * Computes RMS of (raw - MIDSCALE) over RMS_SAMPLES, then converts to mT.
 *
 * Overflow check for uint64_t accumulator:
 *   max |delta| = 2048 (full-scale)
 *   delta^2     = 4,194,304
 *   × 512       = 2,147,483,648  → fits in uint64_t easily ✓
 *
 * Returns magnitude (always ≥ 0).
 */
static uint32_t ac_rms_mt_x100(uint32_t ch) {
    uint64_t sum_sq = 0;
    for (int i = 0; i < RMS_SAMPLES; i++) {
        int32_t delta = (int32_t)adc2_read(ch) - MIDSCALE;
        sum_sq += (uint64_t)((int64_t)delta * delta);
    }
    uint32_t rms_counts = isqrt((uint32_t)(sum_sq / RMS_SAMPLES));
    /* rms_counts is always positive; apply sensitivity directly */
    return (rms_counts * (uint32_t)SENSITIVITY_NUM) / (uint32_t)SENSITIVITY_DEN;
}

/*
 * Also compute RMS of raw counts (for diagnostic output).
 */
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
    if (abs_mt > LINEAR_RANGE_MT_X100) {
        uart_puts(" SAT!");
    }
}

/* ─── Report ────────────────────────────────────────────────────────────────── */

void do_full_report(void) {
    /* ── Instantaneous snapshot ───────────────────────── */
    uint32_t raw1 = adc2_read(2);
    uint32_t raw2 = adc2_read(3);

    uint32_t mv1  = raw_to_mv(raw1);
    uint32_t mv2  = raw_to_mv(raw2);

    int32_t  mt1  = raw_to_mt_x100(raw1);
    int32_t  mt2  = raw_to_mt_x100(raw2);

    /* ── RMS window ───────────────────────────────────── */
    uint32_t rms1_raw   = rms_raw(2);
    uint32_t rms2_raw   = rms_raw(3);

    uint32_t rms1_mt    = ac_rms_mt_x100(2);
    uint32_t rms2_mt    = ac_rms_mt_x100(3);

    uint32_t rms1_mv    = raw_to_mv(rms1_raw);
    uint32_t rms2_mv    = raw_to_mv(rms2_raw);

    /* ── Print ────────────────────────────────────────── */
    uart_puts("=== INSTANT ===\r\n");

    uart_puts("S1(PA1): RAW=");
    uart_putn(raw1);
    uart_puts("  mV=");
    uart_putn(mv1);
    uart_puts("  mT=");
    uart_put_fixed2(mt1);
    print_sat_warning(mt1);
    uart_puts("\r\n");

    uart_puts("S2(PA6): RAW=");
    uart_putn(raw2);
    uart_puts("  mV=");
    uart_putn(mv2);
    uart_puts("  mT=");
    uart_put_fixed2(mt2);
    print_sat_warning(mt2);
    uart_puts("\r\n");

    uart_puts("=== RMS (");
    uart_putn(RMS_SAMPLES);
    uart_puts(" samples, AC only) ===\r\n");

    uart_puts("S1(PA1): RMS_RAW=");
    uart_putn(rms1_raw);
    uart_puts("  RMS_mV=");
    uart_putn(rms1_mv);
    uart_puts("  AC_RMS_mT=");
    uart_put_fixed2((int32_t)rms1_mt);
    uart_puts("\r\n");

    uart_puts("S2(PA6): RMS_RAW=");
    uart_putn(rms2_raw);
    uart_puts("  RMS_mV=");
    uart_putn(rms2_mv);
    uart_puts("  AC_RMS_mT=");
    uart_put_fixed2((int32_t)rms2_mt);
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
        uart_puts("S1=");
        uart_putn(s1);
        uart_puts(" S2=");
        uart_putn(s2);
        uart_puts("\r\n");
    }
    uart_puts("STREAM STOP\r\n");
}

/* ─── Main ──────────────────────────────────────────────────────────────────── */

int main(void) {
    /* 1. Clock — raw registers, always first (see project notes) */
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
    GPIOA->MODER |= (3U << (1*2));    /* PA1 → analog */
    GPIOA->MODER |= (3U << (6*2));    /* PA6 → analog */

    /* 5. USART2: 9600 baud at 16MHz (16000000/9600 = 1666.7 → 1667) */
    USART2->BRR = 1667;
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;

    /* 6. ADC2 clock = SYSCLK (CCIPR ADC12SEL[1:0] = 10) */
    RCC->CCIPR |= RCC_CCIPR_ADC12SEL_1;

    /* 7. ADC voltage regulator startup */
    ADC2->CR = 0;
    ADC2->CR |= ADC_CR_ADVREGEN;
    for (volatile int i = 0; i < 10000; i++);    /* ~20us settle */

    /* 8. Calibrate ADC2 (single-ended, ADCALDIF=0) */
    ADC2->CR |= ADC_CR_ADCAL;
    while (ADC2->CR & ADC_CR_ADCAL);

    /* 9. Enable ADC2 */
    ADC2->CR |= ADC_CR_ADEN;
    while (!(ADC2->ISR & ADC_ISR_ADRDY));

    uart_puts("BOOT OK\r\n");
    uart_puts("DRV5055A1 dual sensor firmware\r\n");
    uart_puts("Sensitivity: 60mV/mT @ 3.3V | Midscale: 1.65V | Range: +/-22mT\r\n");
    uart_puts("Ratiometric: VCC variation cancels in ADC ratio\r\n");
    uart_puts("R=report  A=auto-report  S=stream raw\r\n");
    uart_puts("--------\r\n");

    uint8_t auto_mode = 0;

    while (1) {
        if (USART2->ISR & USART_ISR_RXNE) {
            char c = (char)USART2->RDR;
            if      (c == 'R') { do_full_report(); }
            else if (c == 'A') { auto_mode = !auto_mode;
                                 uart_puts(auto_mode ? "AUTO ON\r\n" : "AUTO OFF\r\n"); }
            else if (c == 'S') { do_stream(); }
        }

        if (auto_mode) {
            do_full_report();
            /* ~200ms delay at 16MHz */
            for (volatile uint32_t d = 0; d < 3200000UL; d++);
        }
    }
}