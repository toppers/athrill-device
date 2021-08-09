#include "hakoniwa_device_run.h"
#include "assert.h"
#include <stdio.h>

#define DEFAULT_CPU_FREQ	100
HakoniwaAssetDeviceControllerType hakoniwa_asset_controller;


static Std_ReturnType hakoniwa_asset_thread_do_init(MpthrIdType id);
static Std_ReturnType hakoniwa_asset_thread_do_proc(MpthrIdType id);
static Std_ReturnType hakoniwa_asset_thread_do_tx_proc(MpthrIdType id);

static MpthrOperationType hakoniwa_asset_op = {
	.do_init = hakoniwa_asset_thread_do_init,
	.do_proc = hakoniwa_asset_thread_do_proc,
};

static MpthrOperationType hakoniwa_asset_tx_op = {
	.do_init = hakoniwa_asset_thread_do_init,
	.do_proc = hakoniwa_asset_thread_do_tx_proc,
};

static int hakoniwa_packet_actuator_raw_encode(const HakoniwaAssetDeviceType* in, HakoniwaPacketBufferType* out)
{
	memcpy(out->datap, (char*)&in->asset_time, sizeof(in->asset_time));
	return sizeof(in->asset_time);
}
void hakoniwa_packet_sensor_raw_decode(const HakoniwaPacketBufferType* in, HakoniwaCoreDeviceType* out)
{
	out->version = 0x1;
	memcpy((char*)&out->hakoniwa_time, in->datap, sizeof(out->hakoniwa_time));
	return;
}

AthrillExDevOperationType *athrill_ex_devop;
void ex_device_init(MpuAddressRegionType *region, AthrillExDevOperationType *athrill_ops)
{
	uint32 portno;
	Std_ReturnType err;
	MpthrIdType thrid;
	athrill_ex_devop = athrill_ops;
	hakoniwa_asset_controller.cpu_freq = DEFAULT_CPU_FREQ; /* 100MHz */
	(void)athrill_ex_devop->param.get_devcfg_value("DEVICE_CONFIG_TIMER_FD", &hakoniwa_asset_controller.cpu_freq);

	hakoniwa_asset_controller.remote_ipaddr = "127.0.0.1";
	(void)athrill_ex_devop->param.get_devcfg_string("HAKONIWA_ASSET_TX_IPADDR", &hakoniwa_asset_controller.remote_ipaddr);

	err = athrill_ex_devop->param.get_devcfg_value("HAKONIWA_ASSET_TX_PORTNO", &portno);
	if (err != STD_E_OK) {
		printf("ERROR: can not load param HAKONIWA_ASSET_TX_PORTNO\n");
		ASSERT(err == STD_E_OK);
	}
	printf("HAKONIWA_ASSET_TX_PORTNO=%d\n", portno);

	hakoniwa_asset_controller.local_ipaddr = "127.0.0.1";
	(void)athrill_ex_devop->param.get_devcfg_string("HAKONIWA_ASSET_RX_IPADDR", &hakoniwa_asset_controller.local_ipaddr);
	hakoniwa_asset_controller.udp_config.client_port = (uint16)portno;
	err = athrill_ex_devop->param.get_devcfg_value("HAKONIWA_ASSET_RX_PORTNO", &portno);
	if (err != STD_E_OK) {
		printf("ERROR: can not load param HAKONIWA_ASSET_RX_PORTNO\n");
		ASSERT(err == STD_E_OK);
	}
	hakoniwa_asset_controller.udp_config.server_port = (uint16)portno;
	printf("HAKONIWA_ASSET_RX_PORTNO=%d\n", portno);

	err = athrill_ex_devop->libs.udp.create_ipaddr(&hakoniwa_asset_controller.udp_config,
			&hakoniwa_asset_controller.udp_comm,
			hakoniwa_asset_controller.local_ipaddr);
	ASSERT(err == STD_E_OK);

	err = athrill_ex_devop->libs.thread.thr_register(&thrid, &hakoniwa_asset_op);
	ASSERT(err == STD_E_OK);
	err = athrill_ex_devop->libs.thread.start_proc(thrid);
	ASSERT(err == STD_E_OK);

	err = athrill_ex_devop->libs.thread.thr_register(&thrid, &hakoniwa_asset_tx_op);
	ASSERT(err == STD_E_OK);
	err = athrill_ex_devop->libs.thread.start_proc(thrid);
	ASSERT(err == STD_E_OK);

	hakoniwa_asset_controller.asset_device.version = 0x1;

	hakoniwa_asset_controller.packet_actuator_encode = hakoniwa_packet_actuator_raw_encode;
	hakoniwa_asset_controller.packet_sensor_decode = hakoniwa_packet_sensor_raw_decode;

	return;
}
static void hakoniwa_send_packet(void);

void ex_device_supply_clock(DeviceClockType *dev_clock)
{
	uint64 interval_ticks;
	uint64 hakoniwa_time_ticks;

	hakoniwa_time_ticks = hakoniwa_asset_controller.core_device.hakoniwa_time * ((uint64)hakoniwa_asset_controller.cpu_freq);
	if (hakoniwa_time_ticks == 0) {
		dev_clock->min_intr_interval = 1U;
	}
	else if (dev_clock->min_intr_interval != DEVICE_CLOCK_MAX_INTERVAL) {
		if ((hakoniwa_time_ticks <= dev_clock->clock)) {
			interval_ticks = 1U;
		}
		else {
			interval_ticks = (hakoniwa_time_ticks - dev_clock->clock);
		}
		if (interval_ticks < dev_clock->min_intr_interval) {
			dev_clock->min_intr_interval = interval_ticks;
		}
	}

	hakoniwa_asset_controller.asset_device.asset_time = dev_clock->clock /  ((uint64)hakoniwa_asset_controller.cpu_freq);

	return;
}

static void hakoniwa_send_packet(void)
{
	Std_ReturnType err;
	HakoniwaPacketBufferType packet;

	packet.datap = hakoniwa_asset_controller.udp_comm.write_data.buffer;
	packet.len = sizeof(hakoniwa_asset_controller.udp_comm.write_data.buffer);
	hakoniwa_asset_controller.udp_comm.write_data.len = hakoniwa_asset_controller.packet_actuator_encode(
    		(const HakoniwaAssetDeviceType *)&hakoniwa_asset_controller.asset_device, &packet);

	err = athrill_ex_devop->libs.udp.remote_write(&hakoniwa_asset_controller.udp_comm, hakoniwa_asset_controller.remote_ipaddr);
	ASSERT(err == STD_E_OK);
	return;
}
static Std_ReturnType hakoniwa_asset_thread_do_tx_proc(MpthrIdType id)
{
	printf("hakoniwa_asset_thread_do_tx_proc:start\n");

	while (1) {
		hakoniwa_send_packet();
		usleep(10 * 1000);
	}
	return STD_E_OK;
}

static Std_ReturnType hakoniwa_asset_thread_do_init(MpthrIdType id)
{
	return STD_E_OK;
}


static Std_ReturnType hakoniwa_asset_thread_do_proc(MpthrIdType id)
{
	Std_ReturnType err;

	printf("hakoniwa_asset_thread_do_proc:start\n");

	while (1) {
		HakoniwaPacketBufferType in;

		err = athrill_ex_devop->libs.udp.read(&hakoniwa_asset_controller.udp_comm);
		if (err != STD_E_OK) {
			continue;
		}
		in.datap = hakoniwa_asset_controller.udp_comm.read_data.buffer;
		in.len = hakoniwa_asset_controller.udp_comm.read_data.len;
		hakoniwa_asset_controller.packet_sensor_decode((const HakoniwaPacketBufferType *)&in, &hakoniwa_asset_controller.core_device);

		//printf("recv: time=%llu\n", hakoniwa_asset_controller.core_device.hakoniwa_time);
		//hakoniwa_send_packet();
	}
	return STD_E_OK;
}
