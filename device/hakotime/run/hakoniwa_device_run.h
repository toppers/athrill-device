#ifndef _HAKONIWA_DEVICE_RUN_H_
#define _HAKONIWA_DEVICE_RUN_H_

#include "athrill_exdev.h"

extern void ex_device_init(MpuAddressRegionType *region, AthrillExDevOperationType *athrill_ops);
extern void ex_device_supply_clock(DeviceClockType *dev_clock);

typedef struct {
    uint32						cpu_freq;
    uint32						timer_fd;
    uint32                      hako_time_only;
} HakoniwaAssetDeviceControllerType;

extern HakoniwaAssetDeviceControllerType hakoniwa_asset_controller;


#endif
