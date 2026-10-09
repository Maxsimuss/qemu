ENTRY(_start)
SECTIONS {
    . = 0x40080000;
    .text : { *(.literal*) *(.text*) }
    . = 0x40081300;
    .level1 : { *(.level1) }
}
