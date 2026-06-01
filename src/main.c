/*
 * pwm_test.c - TIM1 PWM test on PA8 (IN1) and PA9 (IN2)
 * STM32G431CBU6 | Raw registers | 16MHz HSI
 *
 * DRV8874PWPR connections:
 *   PA8  → IN1  (TIM1_CH1, AF6)
 *   PA9  → IN2  (TIM1_CH2, AF6)
 *   3.3V → DVDD, nSLEEP (tie high to wake chip)
 *   GND  → GND
 *   PVDD → leave disconnected for logic-only test
 *
 * DRV8874 PH/EN vs IN/IN mode:
 *   The DRV8874 supports two control modes selected by the PHEN pin.
 *   PHEN low  = IN/IN mode: IN1 and IN2 independently control each half-bridge.
 *   PHEN high = PH/EN mode: IN1=direction, IN2=PWM enable.
 *   This firmware uses IN/IN mode (tie PHEN to GND).
 *
 *   IN/IN truth table:
 *     IN1=0, IN2=0 → Coast (Hi-Z)
 *     IN1=1, IN2=0 → Forward
 *     IN1=0, IN2=1 → Reverse
 *     IN1=1, IN2=1 → Brake
 *
 * PWM strategy for IN/IN mode:
 *   Forward at N% duty: IN1 = N% PWM, IN2 = 0 (constant low)
 *   Reverse at N% duty: IN1 = 0,       IN2 = N% PWM
 *   Brake:              IN1 = 1,        IN2 = 1
 *   Coast:              IN1 = 0,        IN2 = 0
 *
 * Timer config:
 *   TIM1 on APB2, clock = 16MHz HSI (no PLL)
 *   Period = 1000 counts → 16kHz PWM (16MHz / 1000)
 *   CCR1/CCR2 = 0–1000 = 0–100% duty cycle
 *
 * Commands (UART, same port as sensor firmware):
 *   'F' = Forward 50% (IN1 PWM, IN2 low)
 *   'R' = Reverse 50% (IN1 low, IN2 PWM)
 *   'B' = Brake (IN1 high, IN2 high)
 *   'C' = Coast (IN1 low, IN2 low)
 *   '+' = Increase duty 10%
 *   '-' = Decrease duty 10%
 *   'P' = Print current state
 *
 * Verification without 24V:
 *   Multimeter on PA8/PA9: forward → PA8 ~1.65V avg (50% of 3.3V), PA9 = 0V
 *   Oscilloscope preferred: should show clean 16kHz square wave
 *   DRV8874 nFAULT pin will float/high with no PVDD — that's normal
 */

#include "stm32g4xx.h"

/* ─── PWM config ─────────────────────────────────────────────────────────────── */
#define PWM_PERIOD      1000    /* ARR value: 16MHz/1000 = 16kHz                 */
#define PWM_DUTY_STEP   100     /* 10% per +/- press (100/1000 = 10%)            */
#define PWM_DUTY_MAX    1000
#define PWM_DUTY_MIN    0

/* ─── UART (same as sensor firmware) ────────────────────────────────────────── */

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

/* ─── PWM state ──────────────────────────────────────────────────────────────── */

typedef enum { COAST, FORWARD, REVERSE, BRAKE } DriveMode;

static DriveMode current_mode = COAST;
static uint32_t  duty         = 500;   /* start at 50% */

/*
 * Apply current mode and duty to TIM1 CCR1/CCR2.
 *
 * TIM1 CH1 = PA8 = IN1
 * TIM1 CH2 = PA9 = IN2
 *
 * CCRx = 0         → pin stays low  (0% duty)
 * CCRx = PWM_PERIOD → pin stays high (100% duty)
 * CCRx = N          → N/PWM_PERIOD duty cycle
 *
 * PWM mode 1: output high while CNT < CCR, low otherwise.
 */
static void pwm_apply(void) {
    switch (current_mode) {
        case FORWARD:
            /* IN1 = PWM, IN2 = 0 */
            TIM1->CCR1 = duty;
            TIM1->CCR2 = 0;
            break;
        case REVERSE:
            /* IN1 = 0, IN2 = PWM */
            TIM1->CCR1 = 0;
            TIM1->CCR2 = duty;
            break;
        case BRAKE:
            /* IN1 = 1, IN2 = 1 — force both high via CCR = period */
            TIM1->CCR1 = PWM_PERIOD;
            TIM1->CCR2 = PWM_PERIOD;
            break;
        case COAST:
        default:
            /* IN1 = 0, IN2 = 0 */
            TIM1->CCR1 = 0;
            TIM1->CCR2 = 0;
            break;
    }
    /* Trigger update to load shadow registers immediately */
    TIM1->EGR = TIM_EGR_UG;
}

static void print_state(void) {
    const char *mode_str;
    switch (current_mode) {
        case FORWARD: mode_str = "FORWARD"; break;
        case REVERSE: mode_str = "REVERSE"; break;
        case BRAKE:   mode_str = "BRAKE";   break;
        default:      mode_str = "COAST";   break;
    }
    uart_puts("Mode=");
    uart_puts(mode_str);
    uart_puts("  Duty=");
    uart_putn(duty * 100 / PWM_PERIOD);
    uart_puts("%  CCR1=");
    uart_putn(TIM1->CCR1);
    uart_puts("  CCR2=");
    uart_putn(TIM1->CCR2);
    uart_puts("\r\n");
}

/* ─── Main ───────────────────────────────────────────────────────────────────── */

int main(void) {
    /* 1. Clock */
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_HSI;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);

    /* 2. Peripheral clocks */
    RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOAEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
    RCC->APB2ENR  |= RCC_APB2ENR_TIM1EN;      /* TIM1 is on APB2 */

    /* 3. UART pins: PA2=TX, PA3=RX (AF7) */
    GPIOA->MODER &= ~((3U << (2*2)) | (3U << (2*3)));
    GPIOA->MODER |=  ((2U << (2*2)) | (2U << (2*3)));
    GPIOA->AFR[0] &= ~((0xFU << (2*4)) | (0xFU << (3*4)));
    GPIOA->AFR[0] |=  ((7U   << (2*4)) | (7U   << (3*4)));

    /* 4. PWM pins: PA8=TIM1_CH1 (AF6), PA9=TIM1_CH2 (AF6)
     *
     * PA8 and PA9 are in AFR[1] (covers pins 8-15).
     * AFR[1] bit positions: pin 8 → bits [3:0], pin 9 → bits [7:4]
     */
    GPIOA->MODER &= ~((3U << (8*2)) | (3U << (9*2)));
    GPIOA->MODER |=  ((2U << (8*2)) | (2U << (9*2)));   /* AF mode */
    GPIOA->AFR[1] &= ~((0xFU << ((8-8)*4)) | (0xFU << ((9-8)*4)));
    GPIOA->AFR[1] |=  ((6U   << ((8-8)*4)) | (6U   << ((9-8)*4)));   /* AF6 = TIM1 */
    GPIOA->OSPEEDR |= ((3U << (8*2)) | (3U << (9*2)));   /* high speed for PWM */

    /* 5. UART: 9600 baud at 16MHz */
    USART2->BRR = 1667;
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;

    /* 6. TIM1 PWM setup
     *
     * PSC=0: timer clock = 16MHz (no prescaler)
     * ARR=999: period = 1000 counts → f = 16MHz/1000 = 16kHz
     *
     * CCMR1:
     *   OC1M [6:4] = 110 → PWM mode 1 on CH1
     *   OC2M [14:12] = 110 → PWM mode 1 on CH2
     *   OC1PE, OC2PE = 1 → preload enable (shadow registers)
     *
     * CCER:
     *   CC1E, CC2E = 1 → enable CH1 and CH2 outputs
     *
     * BDTR:
     *   MOE = 1 → main output enable (required for TIM1, advanced timer)
     *   Without MOE the outputs stay low regardless of CCR values.
     */
    TIM1->PSC  = 0;
    TIM1->ARR  = PWM_PERIOD - 1;   /* 999 */
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;

    /* PWM mode 1 on CH1 and CH2, preload enabled */
    TIM1->CCMR1 = (6U << 4)  | TIM_CCMR1_OC1PE   /* CH1: PWM mode 1 + preload */
                | (6U << 12) | TIM_CCMR1_OC2PE;   /* CH2: PWM mode 1 + preload */

    /* Enable CH1 and CH2 outputs (active high) */
    TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;

    /* Main output enable — MANDATORY for TIM1 */
    TIM1->BDTR = TIM_BDTR_MOE;

    /* Load ARR and CCR shadow registers, then start */
    TIM1->EGR  = TIM_EGR_UG;
    TIM1->CR1  = TIM_CR1_ARPE | TIM_CR1_CEN;   /* auto-reload preload + enable */

    uart_puts("BOOT OK\r\n");
    uart_puts("DRV8874 PWM test | TIM1 16kHz | IN/IN mode\r\n");
    uart_puts("F=forward  R=reverse  B=brake  C=coast  +=duty+10%  -=duty-10%  P=print\r\n");
    uart_puts("Starting in COAST (both pins low)\r\n");
    uart_puts("--------\r\n");

    while (1) {
        if (USART2->ISR & USART_ISR_RXNE) {
            char c = (char)USART2->RDR;
            switch (c) {
                case 'F': case 'f':
                    current_mode = FORWARD;
                    pwm_apply();
                    uart_puts("-> FORWARD  ");
                    print_state();
                    break;
                case 'R': case 'r':
                    current_mode = REVERSE;
                    pwm_apply();
                    uart_puts("-> REVERSE  ");
                    print_state();
                    break;
                case 'B': case 'b':
                    current_mode = BRAKE;
                    pwm_apply();
                    uart_puts("-> BRAKE  ");
                    print_state();
                    break;
                case 'C': case 'c':
                    current_mode = COAST;
                    pwm_apply();
                    uart_puts("-> COAST  ");
                    print_state();
                    break;
                case '+':
                    if (duty + PWM_DUTY_STEP <= PWM_DUTY_MAX)
                        duty += PWM_DUTY_STEP;
                    pwm_apply();
                    print_state();
                    break;
                case '-':
                    if (duty >= PWM_DUTY_STEP + PWM_DUTY_MIN)
                        duty -= PWM_DUTY_STEP;
                    pwm_apply();
                    print_state();
                    break;
                case 'P': case 'p':
                    print_state();
                    break;
                default:
                    break;
            }
        }
    }
}