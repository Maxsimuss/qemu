/* Independent TCG execution observer. No firmware clock API hooks. */
    .section .text
    .global _start
_start:
    movi a5, 0
    wsr.ps a5
    rsync
    movi a5, 0x40081000
    wsr.vecbase a5
    movi a5, 2
    wsr.intenable a5
    movi a2, 0x3ffb0000
    rsr.prid a4
    extui a4, a4, 13, 1
    slli a4, a4, 2
    add a2, a2, a4
    movi a3, 0
    s32i a3, a2, 0
1:
    addi a3, a3, 1
    s32i a3, a2, 0
    l32i a5, a2, 8
    beqz a5, 1b
    waiti 0
    j 1b

    .section .level1, "ax"
    /* The fixture peer releases its hardware IRQ after this handler runs. */
    movi a5, 0
    s32i a5, a2, 8
    rfe
