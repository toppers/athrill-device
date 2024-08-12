#ifndef _HAKONIWA_DEVICE_RUN_H_
#define _HAKONIWA_DEVICE_RUN_H_

#include "athrill_exdev.h"

extern void ex_device_init(MpuAddressRegionType *region, AthrillExDevOperationType *athrill_ops);
extern void ex_device_supply_clock(DeviceClockType *dev_clock);

typedef struct {
    char*                       asset_name;
    char*                       robo_name;
    uint32                      is_tx_dirty;
    uint32						tx_data;
    uint32						rx_data;
} HakoniwaAssetSampleDeviceControllerType;

extern HakoniwaAssetSampleDeviceControllerType hakoniwa_asset_sample_controller;


#endif
