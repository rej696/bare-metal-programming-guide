// Copyright (c) 2022-2023 Cesanta Software Limited
// SPDX-License-Identifier: MIT

#include "hal.h"

#define BLINK_PERIOD_MS 500  // LED blinking period in millis
#define LOG_PERIOD_MS 1000   // Info log period in millis

uint32_t SystemCoreClock;  // Required by CMSIS. Holds system core cock value
void SystemInit(void) {    // Called automatically by startup code
  clock_init();            // Sets SystemCoreClock
}

static volatile uint64_t s_ticks;  // Milliseconds since boot
void SysTick_Handler(void) {       // SyStick IRQ handler, triggered every 1ms
  s_ticks++;
}

#if 0
static void led_task(void) {  // Blink LED every BLINK_PERIOD_MS
  static uint64_t timer = 0;
  if (timer_expired(&timer, BLINK_PERIOD_MS, s_ticks)) {
    gpio_toggle(LED_PIN);
  }
}

static void log_task(void) {  // Print a log every LOG_PERIOD_MS
  static uint64_t timer = 0;
  if (timer_expired(&timer, LOG_PERIOD_MS, s_ticks)) {
    printf("tick: %5lu, CPU %lu MHz\n", (unsigned long) s_ticks,
           SystemCoreClock / 1000000);
  }
}
#endif

int main(void) {
  gpio_output(LED_PIN);
#if 0
  uart_init(UART_DEBUG, 115200);
#endif
  uint32_t count = 0;

  while (1) {
      if (count >= 1000) {
          gpio_toggle(LED_PIN);
          count = 0;
      }
  }

#if 0
  for (;;) {
    led_task();
    log_task();
  }
#endif

  return 0;
}
