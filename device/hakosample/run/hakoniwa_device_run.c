#include "hakoniwa_device_run.h"
#include "hakoniwa_device_io.h"
#include "assert.h"
#include "hako_client.h"
#include <stdio.h>
#include <pthread.h>

HakoniwaAssetSampleDeviceControllerType hakoniwa_asset_sample_controller;

AthrillExDevOperationType *athrill_ex_devop;
void ex_device_init(MpuAddressRegionType *region, AthrillExDevOperationType *athrill_ops)
{
	athrill_ex_devop = athrill_ops;

	(void)athrill_ex_devop->param.get_devcfg_string("DEBUG_FUNC_HAKO_ASSET_NAME", &hakoniwa_asset_sample_controller.asset_name);
	(void)athrill_ex_devop->param.get_devcfg_string("DEBUG_FUNC_HAKO_ROBO_NAME", &hakoniwa_asset_sample_controller.robo_name);
	int err = hako_client_init(hakoniwa_asset_sample_controller.asset_name);
	if (err != 0) {
		printf("ERROR: HakoAssetSampleDevice can not init hako_client(): %s\n", hakoniwa_asset_sample_controller.asset_name);
		exit(1);
	}
	printf("INFO: HakoAssetSampleDevice is initialized\n");
	return;
}


void ex_device_supply_clock(DeviceClockType *dev_clock)
{
	if (hako_client_is_simulation_mode() != 0) {
		hako_client_notify_write_pdu_done(hakoniwa_asset_sample_controller.asset_name);
		return;
	}
	if (hako_client_is_pdu_created() != 0) {
		return;
	}
	if (hako_client_pdu_is_dirty(hakoniwa_asset_sample_controller.asset_name, hakoniwa_asset_sample_controller.robo_name, HAKO_SAMPLEDEV_CHANNEL_RX_ID) == 0) {
		(void)hako_client_read_pdu(hakoniwa_asset_sample_controller.asset_name, hakoniwa_asset_sample_controller.robo_name, HAKO_SAMPLEDEV_CHANNEL_RX_ID, (char*)&hakoniwa_asset_sample_controller.rx_data, HAKO_SAMPLEDEV_CHANNEL_RX_SIZE);
	}
	hako_client_notify_read_pdu_done(hakoniwa_asset_sample_controller.asset_name);
	if (hakoniwa_asset_sample_controller.is_tx_dirty != 0) {
		(void)hako_client_write_pdu(hakoniwa_asset_sample_controller.asset_name, hakoniwa_asset_sample_controller.robo_name, HAKO_SAMPLEDEV_CHANNEL_TX_ID, (char*)&hakoniwa_asset_sample_controller.tx_data, HAKO_SAMPLEDEV_CHANNEL_TX_SIZE);
		hakoniwa_asset_sample_controller.is_tx_dirty = 0;
	}
	return;
}
