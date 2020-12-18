#include "athrill_exdev.h"
#include "ev3com_private.h"
#include "assert.h"
#include <stdio.h>

static Std_ReturnType ev3com_mmap_get_data8(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 *data);
static Std_ReturnType ev3com_mmap_get_data16(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint16 *data);
static Std_ReturnType ev3com_mmap_get_data32(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint32 *data);
static Std_ReturnType ev3com_mmap_put_data8(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 data);
static Std_ReturnType ev3com_mmap_put_data16(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint16 data);
static Std_ReturnType ev3com_mmap_put_data32(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint32 data);
static Std_ReturnType ev3com_mmap_get_pointer(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 **data);

MpuAddressRegionOperationType	ev3com_mmap_memory_operation = {
		.get_data8 		= 	ev3com_mmap_get_data8,
		.get_data16		=	ev3com_mmap_get_data16,
		.get_data32		=	ev3com_mmap_get_data32,

		.put_data8 		= 	ev3com_mmap_put_data8,
		.put_data16		=	ev3com_mmap_put_data16,
		.put_data32		=	ev3com_mmap_put_data32,

		.get_pointer	= ev3com_mmap_get_pointer,
};

void device_init_ev3com_mmap(MpuAddressRegionType *region)
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

void device_supply_clock_ev3com_mmap(DeviceClockType *dev_clock)
{
	uint64 interval;
	uint64 unity_sim_time;

	uint64 curr_stime;
	memcpy((void*)&curr_stime, &ev3com_control.rx_data[EV3COM_RX_SIM_TIME(EV3COM_SIM_INX_ME)], 8U);
	ev3com_control.vdev_sim_time[EV3COM_SIM_INX_YOU] = curr_stime;

	unity_sim_time = ev3com_control.vdev_sim_time[EV3COM_SIM_INX_YOU] * ((uint64)ev3com_control.cpu_freq);
	ev3com_control.vdev_sim_time[EV3COM_SIM_INX_ME] = ( dev_clock->clock / ((uint64)ev3com_control.cpu_freq) );

	if ((unity_sim_time != 0) && (dev_clock->min_intr_interval != DEVICE_CLOCK_MAX_INTERVAL)) {
		if ((unity_sim_time <= dev_clock->clock)) {
			interval = 2U;
			//printf("UNITY <= MICON:%llu %llu\n", ev3com_control.ev3com_sim_time[EV3COM_SIM_INX_YOU], ev3com_control.ev3com_sim_time[EV3COM_SIM_INX_ME]);
		}
		else {
			//interval = (unity_sim_time - dev_clock->clock) + ((unity_interval_vtime  * ((uint64)ev3com_udp_control.cpu_freq)) / 2);
			interval = (unity_sim_time - dev_clock->clock);
			//printf("UNITY > MICON:%llu %llu\n", ev3com_control.ev3com_sim_time[EV3COM_SIM_INX_YOU], ev3com_control.ev3com_sim_time[EV3COM_SIM_INX_ME]);
		}
		if (interval < dev_clock->min_intr_interval) {
			dev_clock->min_intr_interval = interval;
		}
	}
	memcpy(&ev3com_control.tx_data[EV3COM_TX_SIM_TIME(EV3COM_SIM_INX_ME)],  (void*)&ev3com_control.vdev_sim_time[EV3COM_SIM_INX_ME], 8U);
	memcpy(&ev3com_control.tx_data[EV3COM_TX_SIM_TIME(EV3COM_SIM_INX_YOU)], (void*)&ev3com_control.vdev_sim_time[EV3COM_SIM_INX_YOU], 8U);
	return;
}

static Std_ReturnType ev3com_mmap_get_data8(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 *data)
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
static Std_ReturnType ev3com_mmap_get_data16(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint16 *data)
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
static Std_ReturnType ev3com_mmap_get_data32(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint32 *data)
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
static Std_ReturnType ev3com_mmap_put_data8(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 data)
{

	if (addr == EV3COM_TX_FLAG(0)) {
	}
	else {
		uint32 off = (addr - EV3COM_TX_DATA_BASE) + EV3COM_TX_DATA_BODY_OFF;
		*((uint8*)(&ev3com_control.tx_data[off])) = data;
	}

	return STD_E_OK;
}
static Std_ReturnType ev3com_mmap_put_data16(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint16 data)
{
	uint32 off = (addr - EV3COM_TX_DATA_BASE) + EV3COM_TX_DATA_BODY_OFF;
	*((uint16*)(&ev3com_control.tx_data[off])) = data;
	return STD_E_OK;
}
static Std_ReturnType ev3com_mmap_put_data32(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint32 data)
{
	uint32 off = (addr - EV3COM_TX_DATA_BASE) + EV3COM_TX_DATA_BODY_OFF;
	*((uint32*)(&ev3com_control.tx_data[off])) = data;
	return STD_E_OK;
}
static Std_ReturnType ev3com_mmap_get_pointer(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 **data)
{
	uint32 off = (addr - region->start);
	*data = &region->data[off];
	return STD_E_OK;
}
