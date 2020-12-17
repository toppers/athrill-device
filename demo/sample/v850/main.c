#include "test_serial.h"
#include "section.h"
#include "interrupt.h"
#include <string.h>
#include "timer.h"
#include "v850_ins.h"
#include "athrill_serial_driver.h"
#include "interrupt_table.h"
#include "test_reg.h"

unsigned char stack_data[STACK_SIZE] __attribute__ ((section(".bss_noclr")));

#define SAMPLE_DEV_ADDR	((volatile char*)0xFF100000)
int main(void)
{
	volatile char *p = SAMPLE_DEV_ADDR;

	p[0] = 'H';
	p[1] = 'e';
	p[2] = 'l';
	p[3] = 'l';
	p[4] = 'o';
	p[5] = ' ';
	p[6] = 'W';
	p[7] = 'o';
	p[8] = 'r';
	p[9] = 'l';
	p[10] = 'd';
	p[11] = '\n';
	while (TRUE) {
		test_print((const char*)SAMPLE_DEV_ADDR);
		do_halt();
	}
	return 0;
}

void bss_clear(void)
{
	unsigned char *p = &_bss_start;
	unsigned char *e = &_bss_end;
	for (;p < e; p++) {
		*p = 0;
	}
	return;
}

void data_init(void)
{
	unsigned char *p_rom = &_idata_start;
	unsigned char *e_rom = &_idata_end;
	unsigned char *p_ram = &_data_start;

	for (;p_rom < e_rom; p_ram++, p_rom++) {
		*p_ram = *p_rom;
	}
}

