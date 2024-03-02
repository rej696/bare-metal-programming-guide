#include <stdint.h>
#include <stdbool.h>

#define BIT(x) (1UL << (x))
#define PIN(bank, num) ((((bank) - 'A') << 8) | (num))
#define PINNO(pin) (pin & 0xFF)
#define PINBANK(pin) (pin >> 8)

typedef struct gpio {
    volatile uint32_t MODER;
    volatile uint32_t OTYPER;
    volatile uint32_t OSPEEDR;
    volatile uint32_t PUPDR;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t LCKR;
    volatile uint32_t AFRH;
    volatile uint32_t AFRL;
} gpio_t;

#define GPIO(bank) ((gpio_t *) (0x40020000 + 0x400 * (bank)))

typedef enum {
    GPIO_MODE_INPUT = 0,
    GPIO_MODE_OUTPUT = 1,
    GPIO_MODE_AF = 2,
    GPIO_MODE_ANALOG = 3
} gpio_mode_t;

static inline void gpio_set_mode(uint16_t pin, uint8_t mode)
{
    gpio_t *gpio = GPIO(PINBANK(pin));
    uint32_t n = PINNO(pin);
    gpio->MODER &= ~(0x3U << (n * 2));
    gpio->MODER |= (mode & 0x3U) << (n * 2);
}

static inline void gpio_write(uint16_t pin, bool val)
{
    gpio_t *gpio = GPIO(PINBANK(pin));
    if (val) {
        /* Setting using BSRR enables atomic setting of the ODR register, which is
         * the output data register for the gpio */
        /* gpio->ODR |= (1U << PINNO(pin)); */
        gpio->BSRR |= (1U << PINNO(pin));
    } else {
        /* turning off a gpio is done by setting a "reset" bit which is 16 bits
         * further left than the pin number */
        /* gpio->ODR &= ~(1U << PINNO(pin)); */
        gpio->BSRR |= (1U << PINNO(pin) << 16U);
    }
}

typedef struct rcc {
    volatile uint32_t CR;
    volatile uint32_t PLLCFGR;
    volatile uint32_t CFGR;
    volatile uint32_t CIR;
    volatile uint32_t AHB1RSTR;
    volatile uint32_t AHB2RSTR;
    volatile uint32_t RESERVED0;
    volatile uint32_t RESERVED1;
    volatile uint32_t APB1RSTR;
    volatile uint32_t APB2RSTR;
    volatile uint32_t RESERVED2;
    volatile uint32_t RESERVED3;
    volatile uint32_t AHB1ENR;
    volatile uint32_t AHB2ENR;
    volatile uint32_t RESERVED4;
    volatile uint32_t RESERVED5;
    volatile uint32_t APB1ENR;
    volatile uint32_t APB2ENR;
    volatile uint32_t RESERVED6;
    volatile uint32_t RESERVED7;
    volatile uint32_t AHB1LPENR;
    volatile uint32_t AHB2LPENR;
    volatile uint32_t RESERVED8;
    volatile uint32_t RESERVED9;
    volatile uint32_t APB1LPENR;
    volatile uint32_t APB2LPENR;
    volatile uint32_t RESERVEDA;
    volatile uint32_t RESERVEDB;
    volatile uint32_t BDCR;
    volatile uint32_t CSR;
    volatile uint32_t RESERVEDC;
    volatile uint32_t RESERVEDD;
    volatile uint32_t SSCGR;
    volatile uint32_t PLLI2SCFGR;
    volatile uint32_t DCKCFGR;
} rcc_t;

#define RCC ((rcc_t *) 0x40023800)

typedef struct systick {
    volatile uint32_t CTRL;
    volatile uint32_t LOAD;
    volatile uint32_t VAL;
    volatile uint32_t CALIB;
} systick_t;

#define SYSTICK ((systick_t *) 0xe000e010)

static inline void systick_init(uint32_t const ticks) {
    if ((ticks - 1) > 0xffffff) return; // Systick timer is 24 bit
    SYSTICK->LOAD = ticks - 1;
    SYSTICK->VAL = 0;
    SYSTICK->CTRL = BIT(0) | BIT(1) | BIT(2); /* Enable Systick */
    RCC->APB2ENR |= BIT(14); /* SYSCFG enable */
}

#define CLOCK_FREQ (16000000) /* HSI (internal) clock for black pill is 16MHz */

static volatile uint32_t s_ticks; /* tick counter */
void SysTick_Handler(void) {
    s_ticks++;
}

static bool timer_expired(uint32_t *const timer, uint32_t const period, uint32_t const now)
{
    /* reset timer if wrapped */
    if ((now + period) < (*timer)) {
        *timer = 0;
    }
    /* set expiration if first poll */
    if (*timer == 0) {
        *timer = now + period;
    }
    /* Return if not expired yet */
    if (*timer > now) {
        return false;
    }
    /* Set the next expiration time */
    if ((now - *timer) > period) {
        *timer = now + period;
    } else {
        *timer = *timer + period;
    }
    return true;
}


int main(void)
{
    uint16_t led = PIN('C', 13);
    RCC->AHB1ENR |= BIT(PINBANK(led));
    gpio_set_mode(led, GPIO_MODE_OUTPUT);
    systick_init(CLOCK_FREQ / 1000); /* tick every ms */
    uint32_t timer = 0;
    uint32_t period = 500; /* Toggle LEDs every 500 ms */

    while (1) {
        if (timer_expired(&timer, period, s_ticks)) {
            static bool on = true;
            gpio_write(led, on);
            on = !on;
        }
    }

    return 0;
}

/* Startup code */
__attribute__((naked, noreturn)) void _reset(void)
{
    /* declare linkerscript symbols */
    extern uint32_t _sbss, _ebss; /* Start/end of .bss section */
    extern uint32_t _sdata, _edata; /* Start/end of .data section in flash */
    extern uint32_t _sidata; /* Start of .data section in sram */

    /* set .bss to zero */
    for (uint32_t *dst = &_sbss; dst < &_ebss; ++dst) {
        *dst = 0U;
    }

    for (uint32_t *dst = &_sdata, *src = &_sidata; dst < &_edata; ++dst, ++src) {
        *dst = *src;
    }

    main(); /* call main */

    for (;;) (void) 0; /* Infinite loop if main returns */
}

/* Stack pointer register */
extern void _estack(void);

/* 16 standard and 91 STM32 specific handlers in the vector table */
__attribute__((section(".vectors"))) void (*const tab[16 + 91])(void) = {
    [0] = _estack,
    [1] = _reset,
    [15] = SysTick_Handler
};
