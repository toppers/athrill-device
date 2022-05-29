#include "hakoniwa_device_run.h"
#include "assert.h"
#include <stdio.h>
#include <pthread.h>

#define DEFAULT_CPU_FREQ	100
HakoniwaAssetDeviceControllerType hakoniwa_asset_controller;

typedef struct {
	pthread_mutex_t     mutex;
	pthread_cond_t		cond;
	std_bool			is_wait;
} HakoniwaDeviceSyncType;
static HakoniwaDeviceSyncType hakoniwa_device_sync;


AthrillExDevOperationType *athrill_ex_devop;
void ex_device_init(MpuAddressRegionType *region, AthrillExDevOperationType *athrill_ops)
{
	athrill_ex_devop = athrill_ops;

	hakoniwa_asset_controller.cpu_freq = DEFAULT_CPU_FREQ; /* 100MHz */
	(void)athrill_ex_devop->param.get_devcfg_value("DEVICE_CONFIG_TIMER_FD", &hakoniwa_asset_controller.cpu_freq);


	return;
}


static std_bool hakoniwa_device_supply_clock(DeviceClockType* dev_clock)
{
	uint64 interval_ticks;
	uint64 hakoniwa_time_ticks;
	uint64 hakoniwa_time = 0; //TODO

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
	while (TRUE) {
		std_bool ret = hakoniwa_device_supply_clock(dev_clock);
		//TODO
		uint64 asset_time = 0;
		asset_time = dev_clock->clock / ((uint64)hakoniwa_asset_controller.cpu_freq);
		if (ret == TRUE) {
			break;
		}
		pthread_mutex_lock(&hakoniwa_device_sync.mutex);
		hakoniwa_device_sync.is_wait = TRUE;
		pthread_cond_wait(&hakoniwa_device_sync.cond, &hakoniwa_device_sync.mutex);
		pthread_mutex_unlock(&hakoniwa_device_sync.mutex);
	}

	return;
}
