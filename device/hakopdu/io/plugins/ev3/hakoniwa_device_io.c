#include "hakoniwa_device_io.h"
#include "hakoniwa_ev3.h"
#include "athrill_exdev.h"
#include "assert.h"
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

typedef struct {
	UdpCommConfigType config;
	MpuAddressRegionType *region;
	uint32		cpu_freq;

	/*
	 * for MMAP ONLY
	 */
	uint32		tx_data_addr;
	uint8		*tx_data;
	uint32		rx_data_addr;
	uint8		*rx_data;
} Ev3ComControlType;
static Ev3ComControlType ev3com_control;

void ex_device_memory_init(void)
{
	Std_ReturnType err;
	err = athrill_ex_devop->param.get_devcfg_value_hex("DEBUG_FUNC_EV3COM_MMAP_TX", &ev3com_control.tx_data_addr);
	if (err != STD_E_OK) {
		printf("ERROR: can not load param DEBUG_FUNC_EV3COM_MMAP_TX\n");
	}
	ASSERT(err == STD_E_OK);
	err = athrill_ex_devop->param.get_devcfg_value_hex("DEBUG_FUNC_EV3COM_MMAP_RX", &ev3com_control.rx_data_addr);
	if (err != STD_E_OK) {
		printf("ERROR: can not load param DEBUG_FUNC_EV3COM_MMAP_RX\n");
	}
	ASSERT(err == STD_E_OK);

	err = athrill_ex_devop->dev.get_memory(ev3com_control.tx_data_addr, (uint8**)&ev3com_control.tx_data);
	ASSERT(err == STD_E_OK);

	err = athrill_ex_devop->dev.get_memory(ev3com_control.rx_data_addr, (uint8**)&ev3com_control.rx_data);
	ASSERT(err == STD_E_OK);

	//initialize mmap write buffer header
	{
		Ev3ComTxDataHeadType *tx_headp = (Ev3ComTxDataHeadType*)ev3com_control.tx_data;
		memset((void*)tx_headp, 0, EV3COM_TX_DATA_HEAD_SIZE);
		memcpy((void*)tx_headp->header, EV3COM_TX_DATA_HEAD_HEADER, strlen(EV3COM_TX_DATA_HEAD_HEADER));
		tx_headp->version = EV3COM_TX_DATA_HEAD_VERSION;
		tx_headp->ext_off = EV3COM_TX_DATA_HEAD_EXT_OFF;
		tx_headp->ext_size = EV3COM_TX_DATA_HEAD_EXT_SIZE;
	}
	return;
}

static Std_ReturnType ex_hakoniwadev_get_data8(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 *data)
{
	uint32 off;
	if (addr < EV3COM_TX_DATA_BASE) {
		off = (addr - region->start) + EV3COM_RX_DATA_BODY_OFF;
		*data = *((uint8*)(&ev3com_control.rx_data[off]));
	}
	else {
		off = (addr - EV3COM_TX_DATA_BASE) + EV3COM_TX_DATA_BODY_OFF;
		*data = *((uint8*)(&ev3com_control.tx_data[off]));
	}
	return STD_E_OK;
}

static Std_ReturnType ex_hakoniwadev_get_data16(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint16 *data)
{
	uint32 off;
	if (addr < EV3COM_TX_DATA_BASE) {
		off = (addr - region->start) + EV3COM_RX_DATA_BODY_OFF;
		*data = *((uint16*)(&ev3com_control.rx_data[off]));
	}
	else {
		off = (addr - EV3COM_TX_DATA_BASE) + EV3COM_TX_DATA_BODY_OFF;
		*data = *((uint16*)(&ev3com_control.tx_data[off]));
	}
	return STD_E_OK;
}
static Std_ReturnType ex_hakoniwadev_get_data32(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint32 *data)
{
	uint32 off;
	if (addr < EV3COM_TX_DATA_BASE) {
		off = (addr - region->start) + EV3COM_RX_DATA_BODY_OFF;
		*data = *((uint32*)(&ev3com_control.rx_data[off]));
	}
	else {
		off = (addr - EV3COM_TX_DATA_BASE) + EV3COM_TX_DATA_BODY_OFF;
		*data = *((uint32*)(&ev3com_control.tx_data[off]));
	}
	return STD_E_OK;
}
static Std_ReturnType ex_hakoniwadev_put_data8(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 data)
{
	if (addr == EV3COM_TX_FLAG(0)) {
	}
	else {
		uint32 off = (addr - EV3COM_TX_DATA_BASE) + EV3COM_TX_DATA_BODY_OFF;
		*((uint8*)(&ev3com_control.tx_data[off])) = data;
	}

	return STD_E_OK;
}
static Std_ReturnType ex_hakoniwadev_put_data16(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint16 data)
{
	uint32 off = (addr - EV3COM_TX_DATA_BASE) + EV3COM_TX_DATA_BODY_OFF;
	*((uint16*)(&ev3com_control.tx_data[off])) = data;
	return STD_E_OK;
}
static Std_ReturnType ex_hakoniwadev_put_data32(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint32 data)
{
	uint32 off = (addr - EV3COM_TX_DATA_BASE) + EV3COM_TX_DATA_BODY_OFF;
	*((uint32*)(&ev3com_control.tx_data[off])) = data;
	return STD_E_OK;
}
static Std_ReturnType ex_hakoniwadev_get_pointer(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 **data)
{
	uint32 off = (addr - region->start);
	*data = &region->data[off];
	return STD_E_OK;
}
