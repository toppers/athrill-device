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
extern void athrill_serial_intr_rx(DrvInt32Type channel);
extern void athrill_serial_intr_tx(DrvInt32Type channel);

static void athrill_serial_intr_rx0(void)
{
	athrill_serial_intr_rx(0);
	return;
}
static void athrill_serial_intr_tx0(void)
{
	athrill_serial_intr_tx(0);
	return;
}
#define GPS_DEV_ADDR	0xFF200000
static char buffer[128];

int main(void)
{
	volatile double *easting = (double*)GPS_DEV_ADDR;
	volatile double *northing = (double*)(GPS_DEV_ADDR + 8);
	*easting = 500.0;
	*northing = 1500.0;
	register_interrupt_handler(SERIAL_FIFO_RX_INTNO(0), athrill_serial_intr_rx0);
	register_interrupt_handler(SERIAL_FIFO_TX_INTNO(0), athrill_serial_intr_tx0);
	while (TRUE) {
		int len = athrill_serial_readline(0, buffer, sizeof(buffer));
		buffer[len + 1] = '\0';
		test_print(buffer);
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

