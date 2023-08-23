INCLUDE generated/output_format.ld
ENTRY(_start)

INCLUDE generated/regions.ld

SECTIONS
{
	.text :
	{
		_ftext = .;
                /* Make sure crt0 files come first, and they, and the isr */
                /* don't get disposed of by greedy optimisation */
                *crt0*(.text)
                KEEP(*crt0*(.text))
                KEEP(*(.text.isr))

		FILL(0);
		. = ALIGN(4);
		*(.text .stub .text.* .gnu.linkonce.t.*)
		_etext = .;
	} > rom

	.rodata :
	{
		. = ALIGN(4);
		_frodata = .;
		*(.rodata .rodata.* .gnu.linkonce.r.*)
		*(.rodata1)
		*(.got .got.*)
		*(.toc .toc.*)

		/* Make sure the file is aligned on disk as well
		   as in memory; CRC calculation requires that. */
		FILL(0);
		. = ALIGN(4);
		_erodata = .;
		
	} > rom


	.bss :
	{
		_fbss = .;
		*(.dynsbss)
		*(.sbss .sbss.* .gnu.linkonce.sb.*)
		*(.scommon)
		*(.dynbss)
		*(.bss .bss.* .gnu.linkonce.b.*)
		*(COMMON)
		FILL(0);
		. = ALIGN(4);
		_ebss = .;
		_end = .;
	} > sram

	.data :
	{
		_fdata = .;
		PROVIDE(__global_pointer$ = .);
		*(.data .data.* .gnu.linkonce.d.*)
		*(.data1 .sdata1.*)
		*(.data2 .sdata2.*)
		*(.sdata .sdata.* .gnu.linkonce.s.*)

		/* Make sure the file is aligned on disk as well
		   as in memory; CRC calculation requires that. */
		FILL(0);
		. = ALIGN(4);
		_edata = .;
	} > sram AT > rom
	
	/DISCARD/ :
	{
		*(.eh_frame)
		*(.comment)
	}
}

PROVIDE(_fstack = ORIGIN(sram) + LENGTH(sram) - 8);

PROVIDE(_fdata_rom = LOADADDR(.data));
PROVIDE(_edata_rom = LOADADDR(.data) + SIZEOF(.data));
