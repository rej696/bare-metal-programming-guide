#ifndef GPIO_H_
#define GPIO_H_

#include <stdint.h>
#include <stdbool.h>
#include "rcc.h"
#include "pinutils.h"

typedef struct gpio {
    volatile uint32_t MODER;
    volatile uint32_t OTYPER;
    volatile uint32_t OSPEEDR;
    volatile uint32_t PUPDR;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t LCKR;
    volatile uint32_t AFR[2];
} gpio_t;

#define GPIO(bank) ((gpio_t *) (0x40020000 + 0x400 * (bank)))

typedef enum {
    GPIO_MODE_INPUT = 0,
    GPIO_MODE_OUTPUT = 1,
    GPIO_MODE_AF = 2,
    GPIO_MODE_ANALOG = 3
} gpio_mode_t;

static inline void gpio_set_mode(uint16_t const pin, uint8_t const mode)
{
    gpio_t *gpio = GPIO(PINBANK(pin));
    uint32_t n = PINNO(pin);
    RCC->AHB1ENR |= BIT(PINBANK(pin));
    gpio->MODER &= ~(0x3U << (n * 2));
    gpio->MODER |= (mode & 0x3U) << (n * 2);
}

static inline bool gpio_set_af(uint16_t const pin, uint8_t const af_num)
{
    gpio_t *gpio = GPIO(PINBANK(pin));
    int n = PINNO(pin);
    gpio->AFR[n >> 3] &= ~(15UL << ((n & 7) * 4));
    gpio->AFR[n >> 3] |= ((uint32_t) af_num) << ((n & 7) * 4);
    return true;
}

static inline void gpio_write(uint16_t const pin, bool const val)
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

#endif /* GPIO_H_ */
