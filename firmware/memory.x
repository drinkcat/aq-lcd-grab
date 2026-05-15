MEMORY {
    /*
     * RP2350 has external flash on the Pico 2 W (4 MiB W25Q32RV).
     * 2 MiB is the safe default that matches Embassy's reference; the
     * extra 2 MiB is unused by this firmware.
     */
    FLASH : ORIGIN = 0x10000000, LENGTH = 2048K
    /*
     * 512 KiB SRAM, striped across banks SRAM0-SRAM7 for even load.
     */
    RAM   : ORIGIN = 0x20000000, LENGTH = 512K
    /* Direct-mapped banks for dedicated stacks if we ever go multicore. */
    SRAM8 : ORIGIN = 0x20080000, LENGTH = 4K
    SRAM9 : ORIGIN = 0x20081000, LENGTH = 4K
}

SECTIONS {
    /* Boot ROM / picotool look for IMAGE_DEF in the first 4 KiB of flash. */
    .start_block : ALIGN(4)
    {
        __start_block_addr = .;
        KEEP(*(.start_block));
        KEEP(*(.boot_info));
    } > FLASH

} INSERT AFTER .vector_table;

_stext = ADDR(.start_block) + SIZEOF(.start_block);

SECTIONS {
    .bi_entries : ALIGN(4)
    {
        __bi_entries_start = .;
        KEEP(*(.bi_entries));
        . = ALIGN(4);
        __bi_entries_end = .;
    } > FLASH
} INSERT AFTER .text;

SECTIONS {
    .end_block : ALIGN(4)
    {
        __end_block_addr = .;
        KEEP(*(.end_block));
    } > FLASH

} INSERT AFTER .uninit;

PROVIDE(start_to_end = __end_block_addr - __start_block_addr);
PROVIDE(end_to_start = __start_block_addr - __end_block_addr);
