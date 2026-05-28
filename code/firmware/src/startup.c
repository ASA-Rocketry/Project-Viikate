#include "defs.h"

extern unsigned long _estack;
extern unsigned long _sdata;
extern unsigned long _edata;
extern unsigned long _sdata_load;
extern unsigned long _sbss;
extern unsigned long _ebss;
extern unsigned long _etext;

extern int main(void);

__attribute__((used)) static void _default_handler(void) {
    for (;;) {}
}

__attribute__((used)) void _reset_handler(void) {
    unsigned long *src = &_sdata_load;
    unsigned long *dst = &_sdata;
    while (dst < &_edata) {
        *dst++ = *src++;
    }

    dst = &_sbss;
    while (dst < &_ebss) {
        *dst++ = 0;
    }

    main();

    for (;;) {}
}

void NMI_Handler(void) __attribute__((alias("_default_handler")));
void HardFault_Handler(void) __attribute__((alias("_default_handler")));
void MemManage_Handler(void) __attribute__((alias("_default_handler")));
void BusFault_Handler(void) __attribute__((alias("_default_handler")));
void UsageFault_Handler(void) __attribute__((alias("_default_handler")));

__attribute__((section(".vectors"), used)) void (*const _vectors[])(void) = {
    (void (*)(void)) & _estack,
    _reset_handler,
    NMI_Handler,
    HardFault_Handler,
    MemManage_Handler,
    BusFault_Handler,
    UsageFault_Handler,
};

__attribute__((section(".boot_data"), used))
const unsigned long _boot_data[] = {
    0x60000000,
    (unsigned long)&_etext,
    0,
    0,
};

__attribute__((section(".ivt"), used)) const unsigned long _ivt[] = {
    0x412000D1,
    (unsigned long)&_reset_handler,
    0,
    0,
    (unsigned long)_boot_data,
    0x60000000,
    0,
    0,
};
