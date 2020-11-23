#include "athrill_exdev.h"
#include <stdlib.h>
#include <stdio.h>

static Std_ReturnType gps_get_data8(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 *data);
static Std_ReturnType gps_get_data16(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint16 *data);
static Std_ReturnType gps_get_data32(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint32 *data);
static Std_ReturnType gps_put_data8(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 data);
static Std_ReturnType gps_put_data16(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint16 data);
static Std_ReturnType gps_put_data32(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint32 data);
static Std_ReturnType gps_get_pointer(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 **data);

typedef struct {
	double		easting;
	double		northing;
} AthrillGpsInputType;
static AthrillGpsInputType *athrill_gps_input;
static uint32 athrill_gpsdev_addr;
static AthrillExDevOperationType *athrill_ex_devop;

/**************************************
 * START: external symbols
 **************************************/
int ex_device_memory_size = 1; /* KB */
char ex_device_memory_data[1024];

MpuAddressRegionOperationType	ex_device_memory_operation = {
		.get_data8 		= 	gps_get_data8,
		.get_data16		=	gps_get_data16,
		.get_data32		=	gps_get_data32,

		.put_data8 		= 	gps_put_data8,
		.put_data16		=	gps_put_data16,
		.put_data32		=	gps_put_data32,

		.get_pointer	= gps_get_pointer,
};
void ex_device_init(AthrillExDevOperationType *athrill_ops)
{
	Std_ReturnType err;
	athrill_ex_devop = athrill_ops;
	err = athrill_ex_devop->param.get_devcfg_value_hex("DEBUG_FUNC_GPS_DEVADDR", &athrill_gpsdev_addr);
	if (err != STD_E_OK) {
		printf("ERROR: can not find paramter DEBUG_FUNC_GPS_DEVADDR\n");
		return;
	}

	err = athrill_ex_devop->dev.get_memory(athrill_gpsdev_addr, (uint8**)&athrill_gps_input);
	if (err != STD_E_OK) {
		printf("ERROR: can not find memory addr=0x%x\n", athrill_gpsdev_addr);
		return;
	}

	printf("###INFO gps device addr=0x%x.\n", athrill_gpsdev_addr);
	return;
}

void ex_device_supply_clock(DeviceClockType *dev_clock)
{
	if (dev_clock->clock == 10000) {
		if (athrill_gps_input != NULL) {
			printf("easting=%lf northing=%lf\n", athrill_gps_input->easting, athrill_gps_input->northing);
		}
	}
	return;
}

/**************************************
 * END: external symbols
 **************************************/

static Std_ReturnType gps_get_data8(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 *data)
{
	uint32 off = (addr - region->start);
	*data = *((uint8*)(&region->data[off]));
	return STD_E_OK;
}

static Std_ReturnType gps_get_data16(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint16 *data)
{
	uint32 off = (addr - region->start);
	*data = *((uint16*)(&region->data[off]));
	return STD_E_OK;
}
static Std_ReturnType gps_get_data32(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint32 *data)
{
	uint32 off = (addr - region->start);
	*data = *((uint32*)(&region->data[off]));
	return STD_E_OK;
}
static Std_ReturnType gps_put_data8(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 data)
{
	uint32 off = (addr - region->start);
	*((uint8*)(&region->data[off])) = data;
	return STD_E_OK;
}
static Std_ReturnType gps_put_data16(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint16 data)
{
	uint32 off = (addr - region->start);
	*((uint16*)(&region->data[off])) = data;
	return STD_E_OK;
}
static Std_ReturnType gps_put_data32(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint32 data)
{
	uint32 off = (addr - region->start);
	*((uint32*)(&region->data[off])) = data;
	return STD_E_OK;
}
static Std_ReturnType gps_get_pointer(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 **data)
{
	uint32 off = (addr - region->start);
	*data = &region->data[off];
	return STD_E_OK;
}
