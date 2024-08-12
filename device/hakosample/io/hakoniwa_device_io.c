#include "hakoniwa_device_run.h"
#include "hakoniwa_device_io.h"
#include <stdio.h>

static Std_ReturnType ex_hakoniwadev_get_data8(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 *data);
static Std_ReturnType ex_hakoniwadev_get_data16(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint16 *data);
static Std_ReturnType ex_hakoniwadev_get_data32(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint32 *data);
static Std_ReturnType ex_hakoniwadev_put_data8(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 data);
static Std_ReturnType ex_hakoniwadev_put_data16(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint16 data);
static Std_ReturnType ex_hakoniwadev_put_data32(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint32 data);
static Std_ReturnType ex_hakoniwadev_get_pointer(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 **data);


MpuAddressRegionOperationType	ex_device_memory_operation = {
		.get_data8 		= 	ex_hakoniwadev_get_data8,
		.get_data16		=	ex_hakoniwadev_get_data16,
		.get_data32		=	ex_hakoniwadev_get_data32,

		.put_data8 		= 	ex_hakoniwadev_put_data8,
		.put_data16		=	ex_hakoniwadev_put_data16,
		.put_data32		=	ex_hakoniwadev_put_data32,

		.get_pointer	= ex_hakoniwadev_get_pointer,
};


static Std_ReturnType ex_hakoniwadev_get_data8(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 *data)
{
	uint32 off = (addr - region->start);
	*data = *((uint8*)(&region->data[off]));
	return STD_E_OK;
}

static Std_ReturnType ex_hakoniwadev_get_data16(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint16 *data)
{
	uint32 off = (addr - region->start);
	*data = *((uint16*)(&region->data[off]));
	return STD_E_OK;
}
static Std_ReturnType ex_hakoniwadev_get_data32(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint32 *data)
{
	uint32 off = (addr - region->start);
	if (addr == HAKO_SMAPLEDEV_RX_ADDR) {
		*data = hakoniwa_asset_sample_controller.rx_data;
		//printf("ex_hakoniwadev_get_data32: addr=%u\n", addr);
	}
	else {
		*data = *((uint32*)(&region->data[off]));
	}
	return STD_E_OK;
}
static Std_ReturnType ex_hakoniwadev_put_data8(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 data)
{
	uint32 off = (addr - region->start);
	*((uint8*)(&region->data[off])) = data;
	return STD_E_OK;
}
static Std_ReturnType ex_hakoniwadev_put_data16(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint16 data)
{
	uint32 off = (addr - region->start);
	*((uint16*)(&region->data[off])) = data;
	return STD_E_OK;
}
static Std_ReturnType ex_hakoniwadev_put_data32(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint32 data)
{
	uint32 off = (addr - region->start);
	*((uint32*)(&region->data[off])) = data;
	if (addr == HAKO_SMAPLEDEV_TX_ADDR) {
		printf("ex_hakoniwadev_put_data32: data=%u\n", data);
		hakoniwa_asset_sample_controller.is_tx_dirty = 1;
		hakoniwa_asset_sample_controller.tx_data = data;
	}
	return STD_E_OK;
}
static Std_ReturnType ex_hakoniwadev_get_pointer(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 **data)
{
	uint32 off = (addr - region->start);
	*data = &region->data[off];
	return STD_E_OK;
}
