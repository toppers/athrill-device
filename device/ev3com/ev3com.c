#include "athrill_exdev.h"
#include "ev3com.h"
#include "ev3com_private.h"
#include "std_errno.h"
#include "assert.h"
#include <string.h>
#include <stdio.h>

#define DEFAULT_CPU_FREQ		100 /* MHz */
Ev3ComControlType ev3com_control;
AthrillExDevOperationType *athrill_ex_devop;


static void ex_device_init(MpuAddressRegionType *region, AthrillExDevOperationType *athrill_ops);

#define EX_DEVICE_MEMORY_SIZE		(1024U)
static char ex_device_memory_data[EX_DEVICE_MEMORY_SIZE * 1024];

static MpuAddressRegionOperationType	ev3com_memory_operation;

static void (*device_supply_clock_ev3com_fn) (DeviceClockType *dev_clock);
static void device_supply_clock_ev3com(DeviceClockType *dev_clock)
{
	device_supply_clock_ev3com_fn(dev_clock);
	return;
}
/**************************************
 * START: external symbols
 **************************************/
AthrillExDeviceType athrill_ex_device = {
		.header.magicno = ATHRILL_EXTERNAL_DEVICE_MAGICNO,
		.header.version = ATHRILL_EXTERNAL_DEVICE_VERSION,
		.header.memory_size = EX_DEVICE_MEMORY_SIZE, /* KB */
		.datap = ex_device_memory_data,
		.ops = &ev3com_memory_operation,
		.devinit = ex_device_init,
		.supply_clock = device_supply_clock_ev3com,
};
/**************************************
 * END: external symbols
 **************************************/

static void device_init_ev3com(MpuAddressRegionType *region, Ev3ComIoOperationType op_type)
{
	ev3com_control.region = region;
	ev3com_control.config.is_wait = TRUE;

	ev3com_control.cpu_freq = DEFAULT_CPU_FREQ; /* 100MHz */
	(void)athrill_ex_devop->param.get_devcfg_value("DEVICE_CONFIG_TIMER_FD", &ev3com_control.cpu_freq);

	if (op_type == Ev3ComIoOperation_UDP) {
		ev3com_memory_operation = ev3com_udp_memory_operation;
		device_supply_clock_ev3com_fn = device_supply_clock_ev3com_udp;
		device_init_ev3com_udp(region);
	}
	else if (op_type == Ev3ComIoOperation_MMAP) {
		ev3com_memory_operation = ev3com_mmap_memory_operation;
		device_supply_clock_ev3com_fn = device_supply_clock_ev3com_mmap;
		device_init_ev3com_mmap(region);
	}
	else {
		ASSERT(0);
	}
	return;
}
static void ex_device_init(MpuAddressRegionType *region, AthrillExDevOperationType *athrill_ops)
{
	uint32 enable = 0;
	athrill_ex_devop = athrill_ops;
	athrill_ex_devop->param.get_devcfg_value("DEBUG_FUNC_ENABLE_EV3COM", &enable);
	if (enable != 0) {
		char *sync_type;
		Ev3ComIoOperationType op_type = Ev3ComIoOperation_UDP;
		if (athrill_ex_devop->param.get_devcfg_string("DEBUG_FUNC_EV3COM_SIMSYNC_TYPE", &sync_type) == STD_E_OK) {
			if (strncmp(sync_type, "MMAP", 4) == 0) {
				op_type = Ev3ComIoOperation_MMAP;
			}
		}
		device_init_ev3com(region, op_type);
	}
	return;
}

