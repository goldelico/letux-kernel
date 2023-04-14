OUTPUT_ARCH(mips)
MEMORY{
	/* TOTAL MEM :4KBytes */
	ram   : ORIGIN = 0xb0004000 , LENGTH = 2784
}

ENTRY(fastboot_restore_code_1)
SECTIONS
{
	.text : {
		*(.resume)
		*(.text)
	} > ram

	.data : {
		*(.data)
	        *(.rodata)
	} > ram /**(.data)*/
	.bss : {
		. = ALIGN(4);
			__bss_start = .;
			*(.sbss.*)
			*(.bss.*)
			*(COMMON)
		. = ALIGN(4);
			__bss_end = .;
	} > ram


	/DISCARD/ : {
		*(COMMON)
		*(.pdri)
		*(.comment)
		*(.gnu.attributes)
		*(.reginfo)
	}
}
