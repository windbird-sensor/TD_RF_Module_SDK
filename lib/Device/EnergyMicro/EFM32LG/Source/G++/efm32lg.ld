/* Linker script for Energy Micro EFM32LG devices
 *
 * Version: Sourcery CodeBench Lite 2011.09-69
 * Support: https://support.codesourcery.com/GNUToolchain/
 *
 * Copyright (c) 2007, 2008, 2009, 2010 CodeSourcery, Inc.
 *
 * The authors hereby grant permission to use, copy, modify, distribute,
 * and license this software and its documentation for any purpose, provided
 * that existing copyright notices are retained in all copies and that this
 * notice is included verbatim in any distributions.  No written agreement,
 * license, or royalty fee is required for any of the authorized uses.
 * Modifications to this software may be copyrighted by their authors
 * and need not follow the licensing terms described here, provided that
 * the new terms are clearly indicated on the first page of each file where
 * they apply.
 */

OUTPUT_FORMAT ("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")
ENTRY(__cs3_reset)

MEMORY
{
  /* Declared size is maximum size available. For 128k version a dummy 128k layout is declared at top of flash memory*/
  rom (rx) : ORIGIN = 0x00000000, LENGTH = 256k
  /* Dummy section to let linker calculate size of flash_layout sections*/
  _dummy_ (rw) : ORIGIN = LENGTH(rom), LENGTH = LENGTH(rom)

  ram (rwx) : ORIGIN = 0x20000100, LENGTH = 32k-256
}

/* These force the linker to search for particular symbols from
 * the start of the link process and thus ensure the user's
 * overrides are picked up
 */

EXTERN(__cs3_reset __cs3_reset_em)
EXTERN(__cs3_start_asm _start)
EXTERN(__cs3_stack)
EXTERN(__cs3_reset)

EXTERN(__cs3_interrupt_vector_em)
EXTERN(__cs3_start_c main __cs3_stack __cs3_heap_end)
EXTERN(__compiled_for_flash_size)

/* Provide fall-back values */
PROVIDE(__cs3_heap_start = _end);
PROVIDE(__cs3_heap_end = __cs3_region_start_ram + __cs3_region_size_ram);
PROVIDE(__cs3_region_num = (__cs3_regions_end - __cs3_regions) / 20);
PROVIDE(__cs3_stack = __cs3_region_start_ram + __cs3_region_size_ram);
PROVIDE(__traceram_start = ORIGIN(ram)-256);

SECTIONS
{
  .text :
  {
    CREATE_OBJECT_SYMBOLS
    __cs3_region_start_rom = .;
    *(.cs3.region-head.rom)
    ASSERT (. == __cs3_region_start_rom, ".cs3.region-head.rom not permitted");
    __cs3_interrupt_vector = __cs3_interrupt_vector_em;
    *(.cs3.interrupt_vector)
    /* Make sure we pulled in an interrupt vector.  */
    ASSERT (. != __cs3_interrupt_vector_em, "No interrupt vector");

    PROVIDE(__cs3_reset = __cs3_reset_em);
    *(.cs3.reset)
    PROVIDE(__cs3_start_asm = _start);

    *(.text.cs3.init)
    *(.text .text.* .gnu.linkonce.t.*)
    *(.plt)
    *(.gnu.warning)
    *(.glue_7t) *(.glue_7) *(.vfp11_veneer)

    *(.ARM.extab* .gnu.linkonce.armextab.*)
    *(.gcc_except_table)
  } >rom
  .eh_frame_hdr : ALIGN (4)
  {
    KEEP (*(.eh_frame_hdr))
  } >rom
  .eh_frame : ALIGN (4)
  {
    KEEP (*(.eh_frame))
  } >rom
  /* .ARM.exidx is sorted, so has to go in its own output section.  */
  PROVIDE_HIDDEN (__exidx_start = .);
  .ARM.exidx :
  {
    *(.ARM.exidx* .gnu.linkonce.armexidx.*)
  } >rom
  PROVIDE_HIDDEN (__exidx_end = .);
  .rodata : ALIGN (4)
  {
    *(.rodata .rodata.* .gnu.linkonce.r.*)

    . = ALIGN(4);
    KEEP(*(.init))

    . = ALIGN(4);
    __preinit_array_start = .;
    KEEP (*(.preinit_array))
    __preinit_array_end = .;

    . = ALIGN(4);
    __init_array_start = .;
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array))
    __init_array_end = .;

    . = ALIGN(4);
    KEEP(*(.fini))

    . = ALIGN(4);
    __fini_array_start = .;
    KEEP (*(.fini_array))
    KEEP (*(SORT(.fini_array.*)))
    __fini_array_end = .;

    . = ALIGN(0x4);
    KEEP (*crtbegin.o(.ctors))
    KEEP (*(EXCLUDE_FILE (*crtend.o) .ctors))
    KEEP (*(SORT(.ctors.*)))
    KEEP (*crtend.o(.ctors))

    . = ALIGN(0x4);
    KEEP (*crtbegin.o(.dtors))
    KEEP (*(EXCLUDE_FILE (*crtend.o) .dtors))
    KEEP (*(SORT(.dtors.*)))
    KEEP (*crtend.o(.dtors))

    . = ALIGN(4);
    __cs3_regions = .;
    LONG (0)
    LONG (__cs3_region_init_ram)
    LONG (__cs3_region_start_ram)
    LONG (__cs3_region_init_size_ram)
    LONG (__cs3_region_zero_size_ram)
    __cs3_regions_end = .;
    . = ALIGN (8);
    *(.rom)
    *(.rom.b .bss.rom)
    _etext = .;
  } >rom
  .boot : ALIGN (4)
  {
	 __cs3_start_boot = .;
    *(.boot .boot.*)
  	__cs3_end_boot = .;
  } >rom

  __rom_use = .;

  .data : ALIGN (8)
  {
    __cs3_region_start_ram = .;
    *(.cs3.region-head.ram)
    KEEP(*(.jcr))
    *(.got.plt) *(.got)
    *(.shdata)
    *(.data .data.* .gnu.linkonce.d.*)
    . = ALIGN (8);
    *(.ram)
    . = ALIGN (8);
    _edata = .;
  } >ram AT>rom
  /* __cs3_region_end_rom is deprecated */
  __cs3_region_end_rom = __cs3_region_start_rom + LENGTH(rom);
  __cs3_region_size_rom = .;
  __data_size = . - __cs3_region_start_ram;
  __rom_use = __rom_use + __data_size;

  /* --------------------------------------------- */
  /* Flash layout output                           */
  /* -------------------                           */
  /* Theses "USER" flash zone store                */
  /*    - FlashVariables                           */
  /*    - Sigfox UserSequence                      */
  /*    - Syslog                                   */
  /*    - ...                                      */
  /* --------------------------------------------- */
  /* Theses sections are stored descending from top of flash */
  /* +------------+ -- Declared flash size */
  /* | Flash_dummy|                 */
  /* | Section0   |                 */
  /* +------------+ -- Top of real flash */
  /* | Section1   |                 */
  /* -------------+                 */
  /* | Section2   |                 */
  /* -------------+                 */
  /* | Section3   |                 */
  /* -------------+                 */
  /* | Section4   |                 */
  /* -------------+                 */
  /* | -- Gap --  |                 */
  /* -------------+                 */
  /* | Code,const |                 */
  /* -------------+ -- 0            */
  /* GCC Linker can't calculate size of sections, so we must have a "mirror" of all sections */
  /* theses mirror are in a _dummy_ MEMORY layout - not physical nor real one                */
  /* section content is not outputed on .bin file to not destroy actual value in flash       */
  .flash_layout0i (NOLOAD) :
  {
    KEEP(*(.flash_layout0i))
  } >_dummy_
  .flash_layout1i (NOLOAD) :
  {
    KEEP(*(.flash_layout1i))
  } >_dummy_
  .flash_layout2i (NOLOAD) :
  {
    KEEP(*(.flash_layout2i))
  } >_dummy_
  .flash_layout3i (NOLOAD) :
  {
    KEEP(*(.flash_layout3i))
  } >_dummy_
  .flash_layout4i (NOLOAD) :
  {
    KEEP(*(.flash_layout4i))
  } >_dummy_
  "miss"=(LENGTH(rom) - SIZEOF(.flash_layout0i) - SIZEOF(.flash_layout1i) - SIZEOF(.flash_layout2i) - SIZEOF(.flash_layout3i) - SIZEOF(.flash_layout4i))-__rom_use;
  ASSERT ( __rom_use <=  (LENGTH(rom) - SIZEOF(.flash_layout0i) - SIZEOF(.flash_layout1i) - SIZEOF(.flash_layout2i) - SIZEOF(.flash_layout3i) - SIZEOF(.flash_layout4i)) , "No room available for FLASH LAYOUT data - ROM overflow - please reduce your code size")

  .flash_layout4 LENGTH(rom) - SIZEOF(.flash_layout0i) - SIZEOF(.flash_layout1i) - SIZEOF(.flash_layout2i) - SIZEOF(.flash_layout3i) - SIZEOF(.flash_layout4i) (NOLOAD) :
  {
    KEEP(*(.flash_layout4))
  } >rom
  .flash_layout3 LENGTH(rom) - SIZEOF(.flash_layout0i) - SIZEOF(.flash_layout1i) - SIZEOF(.flash_layout2i) - SIZEOF(.flash_layout3i) (NOLOAD) :
  {
    KEEP(*(.flash_layout3))
  } >rom
  .flash_layout2 LENGTH(rom) - SIZEOF(.flash_layout0i) - SIZEOF(.flash_layout1i) - SIZEOF(.flash_layout2i) (NOLOAD) :
  {
    KEEP(*(.flash_layout2))
  } >rom
  .flash_layout1 LENGTH(rom) - SIZEOF(.flash_layout0i) - SIZEOF(.flash_layout1i) (NOLOAD) :
  {
    KEEP(*(.flash_layout1))
  } >rom
  .flash_layout0 LENGTH(rom) - SIZEOF(.flash_layout0i) (NOLOAD) :
  {
    KEEP(*(.flash_layout0))
  } >rom
	PROVIDE(__compiled_for_flash_size = LENGTH(rom) - SIZEOF(.flash_layout0i));
  /* -------------------------------------- */


  .bss : ALIGN (8)
  {
    *(.bootvar .bootvar.*)
	__cs3_start_bss = .;
    *(.shbss)
    *(.bss .bss.* .gnu.linkonce.b.*)
    *(COMMON)
    . = ALIGN (8);
    *(.ram.b .bss.ram)
    . = ALIGN (8);
    _end = .;
    __end = .;
  } >ram
  .stack (NOLOAD) : ALIGN (8)
  {
	__cs3_end_bss = .;
    *(.stack .stack.*)
    . = ALIGN (8);
  } >ram

  /* __cs3_region_end_ram is deprecated */
  __cs3_region_end_ram = __cs3_region_start_ram + LENGTH(ram);
  __cs3_region_size_ram = LENGTH(ram);
  __cs3_region_init_ram = LOADADDR (.data);
  __cs3_region_init_size_ram = _edata - ADDR (.data);
  __cs3_region_zero_size_ram = _end - _edata;

  __cs3_rodata = LOADADDR (.rodata);


  .stab 0 (NOLOAD) : { *(.stab) }
  .stabstr 0 (NOLOAD) : { *(.stabstr) }
  /* DWARF debug sections.
   * Symbols in the DWARF debugging sections are relative to
   * the beginning of the section so we begin them at 0.
   */
  /* DWARF 1 */
  .debug          0 : { *(.debug) }
  .line           0 : { *(.line) }
  /* GNU DWARF 1 extensions */
  .debug_srcinfo  0 : { *(.debug_srcinfo) }
  .debug_sfnames  0 : { *(.debug_sfnames) }
  /* DWARF 1.1 and DWARF 2 */
  .debug_aranges  0 : { *(.debug_aranges) }
  .debug_pubnames 0 : { *(.debug_pubnames) }
  /* DWARF 2 */
  .debug_info     0 : { *(.debug_info .gnu.linkonce.wi.*) }
  .debug_abbrev   0 : { *(.debug_abbrev) }
  .debug_line     0 : { *(.debug_line) }
  .debug_frame    0 : { *(.debug_frame) }
  .debug_str      0 : { *(.debug_str) }
  .debug_loc      0 : { *(.debug_loc) }
  .debug_macinfo  0 : { *(.debug_macinfo) }
  /* DWARF 2.1 */
  .debug_ranges   0 : { *(.debug_ranges) }
  /* SGI/MIPS DWARF 2 extensions */
  .debug_weaknames 0 : { *(.debug_weaknames) }
  .debug_funcnames 0 : { *(.debug_funcnames) }
  .debug_typenames 0 : { *(.debug_typenames) }
  .debug_varnames  0 : { *(.debug_varnames) }

  .note.gnu.arm.ident 0 : { KEEP (*(.note.gnu.arm.ident)) }
  .ARM.attributes 0 : { KEEP (*(.ARM.attributes)) }
  /DISCARD/ : { *(.note.GNU-stack) }
}
