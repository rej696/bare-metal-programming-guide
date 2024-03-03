#include <stdint.h>
#include <stdbool.h>
#include "pinutils.h"
#include "gpio.h"
#include "rcc.h"

static inline void spin(volatile uint32_t count) {
    while (count--) { (void) 0; }
}

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

typedef struct uart {
    volatile uint32_t SR;
    volatile uint32_t DR;
    volatile uint32_t BRR;
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t CR3;
    volatile uint32_t GTPR;
} uart_t;

typedef enum uart_id {
    UART1 = 0,
    UART2 = 1,
    UART6 = 2
} uart_id_t;

static uart_t *uart_map[3] = {
    [UART1] = ((uart_t *) 0x40011000), /* USART 1 */
    [UART2] = ((uart_t *) 0x40004400), /* USART 2 */
    [UART6] = ((uart_t *) 0x40011400) /* USART 6 */
};

static inline void uart_init(uart_id_t const uart_id, uint32_t const baud) {
    uint8_t af = 0; /* Alternate Function */
    uint16_t rx = 0;
    uint16_t tx = 0;
    switch (uart_id) {
        case UART1: {
            RCC->APB2ENR |= BIT(4);
            tx = PIN('A', 9);
            rx = PIN('A', 10);
            af = 7;
            break;
        }
        case UART2: {
            RCC->APB1ENR |= BIT(17);
            tx = PIN('A', 2);
            rx = PIN('A', 3);
            af = 7;
            break;
        }
        case UART6: {
            RCC->APB2ENR |= BIT(5);
            tx = PIN('A', 11);
            rx = PIN('A', 12);
            af = 8;
            break;
        }
    }

    gpio_set_mode(tx, GPIO_MODE_AF);
    gpio_set_af(tx, af);
    gpio_set_mode(rx, GPIO_MODE_AF);
    gpio_set_af(rx, af);
    uart_map[uart_id]->CR1 = 0;
    uart_map[uart_id]->BRR = CLOCK_FREQ / baud;
    /* 13 = uart enable, 3 = transmit enable, 2 = receive enable */
    uart_map[uart_id]->CR1 |= BIT(13) | BIT(3) | BIT(2);
}

static inline bool uart_read_ready(uart_id_t const uart_id) {
    return uart_map[uart_id]->SR & BIT(5); /* Data is ready if RXNE bit is set */
}

static inline uint8_t uart_read_byte(uart_id_t const uart_id) {
    return (uint8_t)(uart_map[uart_id]->DR & 0xFF);
}

static inline void uart_write_byte(uart_id_t const uart_id, uint8_t const byte) {
    uart_map[uart_id]->DR = byte;
    while ((uart_map[uart_id]->SR & BIT(7)) == 0) { spin(1); }
}

static inline void uart_write_str(uart_id_t const uart_id, char const *const str) {
    for (uint32_t i = 0; str[i] != 0; ++i) {
        uart_write_byte(uart_id, (uint8_t)str[i]);
    }
}

static inline void uart_write_buf(uart_id_t const uart_id, uint32_t const size, uint8_t const buf[size]) {
    for (uint32_t i = 0; i < size; ++i) {
        uart_write_byte(uart_id, buf[i]);
    }
}

int main(void)
{
    uint16_t led = PIN('C', 13);
    gpio_set_mode(led, GPIO_MODE_OUTPUT);
    systick_init(CLOCK_FREQ / 1000); /* tick every ms */
    uint32_t timer = 0;
    uint32_t period = 500; /* Toggle LEDs every 500 ms */
    uart_init(UART1, 9600);

    while (1) {
        if (timer_expired(&timer, period, s_ticks)) {
            static bool on = true;
            gpio_write(led, on);
            on = !on;
            uart_write_str(UART1, "hi\r\n");
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
