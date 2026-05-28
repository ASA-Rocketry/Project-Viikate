MEMORY
{
    ITCM  (rwx) : ORIGIN = 0x00000000, LENGTH = 512K
    DTCM  (rwx) : ORIGIN = 0x20000000, LENGTH = 512K
    OCRAM (rwx) : ORIGIN = 0x20200000, LENGTH = 512K
    FLASH (rx)  : ORIGIN = 0x60000000, LENGTH = 1984K
}

_estack = ORIGIN(DTCM) + LENGTH(DTCM);

SECTIONS
{
    .ivt : {
        KEEP(*(.ivt))
    } > FLASH

    .boot_data : {
        KEEP(*(.boot_data))
    } > FLASH

    .text : {
        . = ALIGN(16);
        _stext = .;
        KEEP(*(.vectors))
        *(.text*)
        *(.rodata*)
        . = ALIGN(4);
        _etext = .;
    } > FLASH

    .data : {
        . = ALIGN(4);
        _sdata = .;
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } > DTCM AT > FLASH

    _sdata_load = LOADADDR(.data);

    .bss : {
        . = ALIGN(4);
        _sbss = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    } > DTCM

    /DISCARD/ : {
        *(.ARM.exidx*)
        *(.ARM.attributes*)
        *(.comment*)
    }
}
