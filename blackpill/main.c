
/* Startup code */
__attribute__((naked, noreturn)) void _reset(void) {
  for (;;) {
    asm("nop");
  };
}

/* Stack pointer register */
extern void _estack(void);

/* 16 standard and 91 STM32 specific handlers in the vector table */
__attribute__((section(".vectors"))) void (*const tab[16 + 91])(void) = {
    _estack,
    _reset
};
