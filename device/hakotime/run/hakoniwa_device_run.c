#include "hakoniwa_device_run.h"
#include "assert.h"
#include "hako_client.h"
#include <stdio.h>
#include <pthread.h>

#define DEFAULT_CPU_FREQ	100
HakoniwaAssetDeviceControllerType hakoniwa_asset_controller;

AthrillExDevOperationType *athrill_ex_devop;
void ex_device_init(MpuAddressRegionType *region, AthrillExDevOperationType *athrill_ops)
{
	char* asset_name = NULL;;
	athrill_ex_devop = athrill_ops;

	hakoniwa_asset_controller.cpu_freq = DEFAULT_CPU_FREQ; /* 100MHz */
	(void)athrill_ex_devop->param.get_devcfg_value("DEVICE_CONFIG_TIMER_FD", &hakoniwa_asset_controller.cpu_freq);
	(void)athrill_ex_devop->param.get_devcfg_string("DEBUG_FUNC_HAKO_ASSET_NAME", &asset_name);
	int err = hako_client_init(asset_name);
	if (err != 0) {
		printf("ERROR: can not init hako_client(): %s\n", asset_name);
		exit(1);
	}
	return;
}


static std_bool hakoniwa_device_supply_clock(DeviceClockType* dev_clock)
{
	uint64 interval_ticks;
	uint64 hakoniwa_time_ticks;

	while (TRUE) {
		if ((hako_client_is_simulation_mode() == 0) && (hako_client_is_pdu_created() == 0)) {
			break;
		}
		usleep(1000 * 100);
    }
	uint64 hakoniwa_time = (uint64)hako_client_get_worldtime();

	hakoniwa_time_ticks = hakoniwa_time * ((uint64)hakoniwa_asset_controller.cpu_freq);
	if (hakoniwa_time_ticks == 0) {
		//printf("CAN NOT STEP: hakoniwa_time_ticks=0\n");
		return FALSE;
	}
	else if (dev_clock->min_intr_interval != DEVICE_CLOCK_MAX_INTERVAL) {
		if ((hakoniwa_time_ticks <= dev_clock->clock)) {
			//printf("CAN NOT STEP: hakoniwa_time_ticks=%lld dev_clock->clock=%lld\n", hakoniwa_time_ticks, dev_clock->clock);
			return FALSE;
		}
		else {
			interval_ticks = (hakoniwa_time_ticks - dev_clock->clock);
		}
		if (interval_ticks < dev_clock->min_intr_interval) {
			dev_clock->min_intr_interval = interval_ticks;
		}
	}

	return TRUE;
}


void ex_device_supply_clock(DeviceClockType *dev_clock)
{
	std_bool ret = hakoniwa_device_supply_clock(dev_clock);
	if (ret == FALSE) {
		dev_clock->min_intr_interval = 1U;
	}
	uint64 sim_time = ( dev_clock->clock / ((uint64)hakoniwa_asset_controller.cpu_freq) );
	hako_client_notify_simtime((hako_time_t)sim_time);
	return;
}
