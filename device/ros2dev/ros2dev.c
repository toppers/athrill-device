#include "athrill_exdev.h"
#include "athrill_priv.h"
#include <stdlib.h>
#include <stdio.h>

#include "assert.h"
#include "ros_dev.h"

static Std_ReturnType ex_extdev_get_data8(MpuAddressRegionType* region, CoreIdType core_id, uint32 addr, uint8* data);
static Std_ReturnType ex_extdev_get_data16(MpuAddressRegionType* region, CoreIdType core_id, uint32 addr, uint16* data);
static Std_ReturnType ex_extdev_get_data32(MpuAddressRegionType* region, CoreIdType core_id, uint32 addr, uint32* data);
static Std_ReturnType ex_extdev_put_data8(MpuAddressRegionType* region, CoreIdType core_id, uint32 addr, uint8 data);
static Std_ReturnType ex_extdev_put_data16(MpuAddressRegionType* region, CoreIdType core_id, uint32 addr, uint16 data);
static Std_ReturnType ex_extdev_put_data32(MpuAddressRegionType* region, CoreIdType core_id, uint32 addr, uint32 data);
static Std_ReturnType ex_extdev_get_pointer(MpuAddressRegionType* region, CoreIdType core_id, uint32 addr, uint8** data);

static MpuAddressRegionOperationType	ex_device_memory_operation = {
		.get_data8 = ex_extdev_get_data8,
		.get_data16 = ex_extdev_get_data16,
		.get_data32 = ex_extdev_get_data32,

		.put_data8 = ex_extdev_put_data8,
		.put_data16 = ex_extdev_put_data16,
		.put_data32 = ex_extdev_put_data32,

		.get_pointer = ex_extdev_get_pointer,
};
AthrillExDevOperationType* athrill_ex_devop;
static void ex_device_init(MpuAddressRegionType* region, AthrillExDevOperationType* athrill_ops)
{
	athrill_ex_devop = athrill_ops;
	printf("SAMPLE_DEVICE: init\n");
	return;
}

static void ex_device_supply_clock(DeviceClockType* dev_clock)
{
	//nothing to do
	return;
}
#define EX_DEVICE_MEMORY_SIZE	(2U * 1024U) /* Bytes */
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

static Std_ReturnType ex_extdev_get_data8(MpuAddressRegionType* region, CoreIdType core_id, uint32 addr, uint8* data)
{
	uint32 off = (addr - region->start);
	*data = *((uint8*)(&region->data[off]));
	return STD_E_OK;
}

static Std_ReturnType ex_extdev_get_data16(MpuAddressRegionType* region, CoreIdType core_id, uint32 addr, uint16* data)
{
	uint32 off = (addr - region->start);
	*data = *((uint16*)(&region->data[off]));
	return STD_E_OK;
}
static Std_ReturnType ex_extdev_get_data32(MpuAddressRegionType* region, CoreIdType core_id, uint32 addr, uint32* data)
{
	uint32 off = (addr - region->start);
	*data = *((uint32*)(&region->data[off]));
	return STD_E_OK;
}
static Std_ReturnType ex_extdev_put_data8(MpuAddressRegionType* region, CoreIdType core_id, uint32 addr, uint8 data)
{
	uint32 off = (addr - region->start);
	*((uint8*)(&region->data[off])) = data;
	return STD_E_OK;
}
static Std_ReturnType ex_extdev_put_data16(MpuAddressRegionType* region, CoreIdType core_id, uint32 addr, uint16 data)
{
	uint32 off = (addr - region->start);
	*((uint16*)(&region->data[off])) = data;
	return STD_E_OK;
}
static Std_ReturnType ex_extdev_put_data32(MpuAddressRegionType* region, CoreIdType core_id, uint32 addr, uint32 data)
{
	RosReqType req;
	uint8* ptr;
	
	Std_ReturnType err = athrill_ex_devop->dev.get_memory(data, (uint8**)&ptr);
	ASSERT(err == STD_E_OK);

	req.id =		*((uint32*)ptr);
	req.ret =		*((uint32*)(ptr + 4U));
	req.datalen =	*((uint32*)(ptr + 8U));
	err = athrill_ex_devop->dev.get_memory((*((uint32*)(ptr + 12U))), (uint8**)&ptr);
	ASSERT(err == STD_E_OK);

	printf("data=0x%x id=%d\n", data, req.id);
	printf("ret=%d\n", req.ret);
	printf("datalen=%d\n", req.datalen);
	printf("ptr_value=%d\n", *((sint32*)ptr));
	return STD_E_OK;
}
static Std_ReturnType ex_extdev_get_pointer(MpuAddressRegionType* region, CoreIdType core_id, uint32 addr, uint8** data)
{
	uint32 off = (addr - region->start);
	*data = &region->data[off];
	return STD_E_OK;
}