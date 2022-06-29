#include "hakoniwa_device_run.h"
#include "hakoniwa_device_io.h"
#include "assert.h"
#include "hako_client.h"
#include <stdio.h>
#include <pthread.h>

#define DEFAULT_CPU_FREQ	100
HakoniwaAssetDeviceControllerType hakoniwa_asset_controller;

AthrillExDevOperationType *athrill_ex_devop;
char* ex_device_hakopdu_asset_name = NULL;
void ex_device_init(MpuAddressRegionType *region, AthrillExDevOperationType *athrill_ops)
{
	athrill_ex_devop = athrill_ops;

	(void)athrill_ex_devop->param.get_devcfg_string("DEBUG_FUNC_HAKO_ASSET_NAME", &ex_device_hakopdu_asset_name);
	int err = hako_client_init(ex_device_hakopdu_asset_name);
	if (err != 0) {
		printf("ERROR: can not init hako_client(): %s\n", ex_device_hakopdu_asset_name);
		exit(1);
	}
	ex_device_memory_init();
	return;
}


void ex_device_supply_clock(DeviceClockType *dev_clock)
{
	ex_device_memory_supply_clock();
	return;
}
