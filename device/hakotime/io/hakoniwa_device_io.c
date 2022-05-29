#include "hakoniwa_device_io.h"

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
	*data = *((uint32*)(&region->data[off]));
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
	return STD_E_OK;
}
static Std_ReturnType ex_hakoniwadev_get_pointer(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 **data)
{
	uint32 off = (addr - region->start);
	*data = &region->data[off];
	return STD_E_OK;
}
