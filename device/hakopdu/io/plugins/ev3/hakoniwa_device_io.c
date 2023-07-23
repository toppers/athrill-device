#include "hakoniwa_device_io.h"
#include "hakoniwa_ev3.h"
#include "athrill_exdev.h"
#include "assert.h"
#include "hako_client.h"
#include "ev3packet.h"
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
	HakoPduChannelIdType		tx_channel_id;
	uint8						tx_data[EV3COM_TX_DATA_SIZE];
	uint8						tx_packet_data[EV3COM_TX_DATA_COMM_SIZE];
	std_bool					is_tx_dirty;
	HakoPduChannelIdType		rx_channel_id;
	uint8						rx_data[EV3COM_RX_DATA_SIZE];
	uint8						rx_packet_data[EV3COM_RX_DATA_COMM_SIZE];
} Ev3ComControlType;
static Ev3ComControlType ev3com_control;

void ex_device_memory_init(void)
{
	Std_ReturnType err;
	ev3com_control.is_tx_dirty = FALSE;
	err = athrill_ex_devop->param.get_devcfg_value("DEBUG_FUNC_EV3COM_CHANNEL_ID_TX", (unsigned int*)&ev3com_control.tx_channel_id);
	if (err != STD_E_OK) {
		printf("ERROR: can not load param DEBUG_FUNC_EV3COM_CHANNEL_ID_TX\n");
	}
	ASSERT(err == STD_E_OK);
	err = athrill_ex_devop->param.get_devcfg_value("DEBUG_FUNC_EV3COM_CHANNEL_ID_RX", (unsigned int*)&ev3com_control.rx_channel_id);
	if (err != STD_E_OK) {
		printf("ERROR: can not load param DEBUG_FUNC_EV3COM_CHANNEL_ID_RX\n");
	}
	ASSERT(err == STD_E_OK);

	//initialize write buffer header
	{
		Ev3ComTxDataHeadType *tx_headp = (Ev3ComTxDataHeadType*)ev3com_control.tx_data;
		memset((void*)tx_headp, 0, EV3COM_TX_DATA_HEAD_SIZE);
		memcpy((void*)tx_headp->header, EV3COM_TX_DATA_HEAD_HEADER, strlen(EV3COM_TX_DATA_HEAD_HEADER));
		tx_headp->version = EV3COM_TX_DATA_HEAD_VERSION;
		tx_headp->ext_off = EV3COM_TX_DATA_HEAD_EXT_OFF;
		tx_headp->ext_size = EV3COM_TX_DATA_HEAD_EXT_SIZE;
	}
	//err = hako_client_create_pdu_channel(ev3com_control.tx_channel_id, EV3COM_TX_DATA_COMM_SIZE);
	//ASSERT(err == 0);
	return;
}
void ex_device_memory_supply_clock(void)
{
    if (hako_client_is_pdu_created() != 0) {
		/* nothing to do */
	}
    else if (hako_client_is_simulation_mode() == 0) {
		if (hako_client_pdu_is_dirty(ex_device_hakopdu_asset_name, ex_device_hakopdu_robo_name, ev3com_control.rx_channel_id) == 0) {
			(void)hako_client_read_pdu(ex_device_hakopdu_asset_name, ex_device_hakopdu_robo_name, ev3com_control.rx_channel_id, (char*)ev3com_control.rx_packet_data, EV3COM_RX_DATA_COMM_SIZE);
			ev3packet_sensor_decode((const Hako_Ev3PduSensor *)ev3com_control.rx_packet_data, (char*)ev3com_control.rx_data);
		}
        hako_client_notify_read_pdu_done(ex_device_hakopdu_asset_name);
		if (ev3com_control.is_tx_dirty == TRUE) {
			ev3packet_actuator_encode((const char*)ev3com_control.tx_data, (Hako_Ev3PduActuator *)ev3com_control.tx_packet_data);
			(void)hako_client_write_pdu(ex_device_hakopdu_asset_name, ex_device_hakopdu_robo_name, ev3com_control.tx_channel_id, (char*)ev3com_control.tx_packet_data, EV3COM_TX_DATA_COMM_SIZE);
		}
    }
    else if (hako_client_is_pdu_sync_mode(ex_device_hakopdu_asset_name) == 0) {
		ev3packet_actuator_encode((const char*)ev3com_control.tx_data, (Hako_Ev3PduActuator *)ev3com_control.tx_packet_data);
		(void)hako_client_write_pdu(ex_device_hakopdu_asset_name, ex_device_hakopdu_robo_name, ev3com_control.tx_channel_id, (char*)ev3com_control.tx_packet_data, EV3COM_TX_DATA_COMM_SIZE);
        hako_client_notify_write_pdu_done(ex_device_hakopdu_asset_name);
		ev3com_control.is_tx_dirty = FALSE;
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
		ev3com_control.is_tx_dirty = TRUE;
	}

	return STD_E_OK;
}
static Std_ReturnType ex_hakoniwadev_put_data16(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint16 data)
{
	uint32 off = (addr - EV3COM_TX_DATA_BASE) + EV3COM_TX_DATA_BODY_OFF;
	*((uint16*)(&ev3com_control.tx_data[off])) = data;
	ev3com_control.is_tx_dirty = TRUE;
	return STD_E_OK;
}
static Std_ReturnType ex_hakoniwadev_put_data32(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint32 data)
{
	uint32 off = (addr - EV3COM_TX_DATA_BASE) + EV3COM_TX_DATA_BODY_OFF;
	*((uint32*)(&ev3com_control.tx_data[off])) = data;
	ev3com_control.is_tx_dirty = TRUE;
	return STD_E_OK;
}
static Std_ReturnType ex_hakoniwadev_get_pointer(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 **data)
{
	uint32 off = (addr - region->start);
	*data = &region->data[off];
	return STD_E_OK;
}
