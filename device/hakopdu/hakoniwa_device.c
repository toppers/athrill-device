#include "run/hakoniwa_device_run.h"
#include "io/hakoniwa_device_io.h"

#define EX_DEVICE_MEMORY_SIZE	(1U * 1024U) /* Bytes */
static char ex_device_memory_data[EX_DEVICE_MEMORY_SIZE];

/**************************************
 * START: external symbols
 **************************************/
AthrillExDeviceType athrill_ex_device = {
		.header.magicno = ATHRILL_EXTERNAL_DEVICE_MAGICNO,
		.header.version = ATHRILL_EXTERNAL_DEVICE_VERSION,
		.header.memory_size = EX_DEVICE_MEMORY_SIZE, /* Bytes */
		.datap = ex_device_memory_data,
		.ops = &ex_device_memory_operation,
		.devinit = ex_device_init,
		.supply_clock = ex_device_supply_clock,
};
/**************************************
 * END: external symbols
 **************************************/

