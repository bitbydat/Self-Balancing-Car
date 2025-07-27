ENTRY(Reset_Handler)

MEMORY
{
	FLASH(rx):ORIGIN =0x08000000,LENGTH =64K
	SRAM(rwx):ORIGIN =0x20000000,LENGTH =20K
}

__max_heap_size = 0x400;	/* A symbol declaration */
__max_stack_size = 0x200;

SECTIONS
{
	.text :
	{	/* '.' = VMA = CD0x08000000 */
		*(.isr_vector)
		*(.text)
		*(.rodata)
		. = ALIGN(4);
		_etext = .;		/* store the updated location counter value into a symbol '_etext' */	
	}> FLASH
	
	.data :
	{
		. = ALIGN(4);
    	_sdata = .;        /* create a global symbol at data start */
    	*(.data)           /* .data sections */
    	*(.data*)          /* .data* sections */
    	. = ALIGN(4);
    	_edata = .;        /* define a global symbol at data end */
	}> SRAM AT> FLASH
	
	.bss :
	{
		_sbss = .;
		*(.bss)
		*(COMMON)
		. = ALIGN(4);
		_ebss = .;
	}> SRAM
}
