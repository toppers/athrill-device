#include "athrill_exdev.h"
#include <stdlib.h>
#include <stdio.h>

static Std_ReturnType ex_sampledev_get_data8(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 *data);
static Std_ReturnType ex_sampledev_get_data16(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint16 *data);
static Std_ReturnType ex_sampledev_get_data32(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint32 *data);
static Std_ReturnType ex_sampledev_put_data8(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 data);
static Std_ReturnType ex_sampledev_put_data16(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint16 data);
static Std_ReturnType ex_sampledev_put_data32(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint32 data);
static Std_ReturnType ex_sampledev_get_pointer(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 **data);

/**************************************
 * START: external symbols
 **************************************/
int ex_device_memory_size = 2; /* KB */
char ex_device_memory_data[2048];

MpuAddressRegionOperationType	ex_device_memory_operation = {
		.get_data8 		= 	ex_sampledev_get_data8,
		.get_data16		=	ex_sampledev_get_data16,
		.get_data32		=	ex_sampledev_get_data32,

		.put_data8 		= 	ex_sampledev_put_data8,
		.put_data16		=	ex_sampledev_put_data16,
		.put_data32		=	ex_sampledev_put_data32,

		.get_pointer	= ex_sampledev_get_pointer,
};
static AthrillExDevOperationType *athrill_ex_devop;
void ex_device_init(AthrillExDevOperationType *athrill_ops)
{
	athrill_ex_devop = athrill_ops;
	printf("###INFO sample device init success.\n");
	return;
}

void ex_device_supply_clock(DeviceClockType *dev_clock)
{
	if (dev_clock->clock == 10000) {
		printf("OK! sample device supply clock is good. clocks=%lld\n", dev_clock->clock);
	}
	return;
}

/**************************************
 * END: external symbols
 **************************************/

static Std_ReturnType ex_sampledev_get_data8(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 *data)
{
	uint32 off = (addr - region->start);
	*data = *((uint8*)(&region->data[off]));
	return STD_E_OK;
}

static Std_ReturnType ex_sampledev_get_data16(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint16 *data)
{
	uint32 off = (addr - region->start);
	*data = *((uint16*)(&region->data[off]));
	return STD_E_OK;
}
static Std_ReturnType ex_sampledev_get_data32(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint32 *data)
{
	uint32 off = (addr - region->start);
	*data = *((uint32*)(&region->data[off]));
	return STD_E_OK;
}
static Std_ReturnType ex_sampledev_put_data8(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 data)
{
	uint32 off = (addr - region->start);
	*((uint8*)(&region->data[off])) = data;
	return STD_E_OK;
}
static Std_ReturnType ex_sampledev_put_data16(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint16 data)
{
	uint32 off = (addr - region->start);
	*((uint16*)(&region->data[off])) = data;
	return STD_E_OK;
}
static Std_ReturnType ex_sampledev_put_data32(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint32 data)
{
	uint32 off = (addr - region->start);
	*((uint32*)(&region->data[off])) = data;
	return STD_E_OK;
}
static Std_ReturnType ex_sampledev_get_pointer(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 **data)
{
	uint32 off = (addr - region->start);
	*data = &region->data[off];
	return STD_E_OK;
}
