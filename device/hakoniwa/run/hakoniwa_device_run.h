#ifndef _HAKONIWA_DEVICE_RUN_H_
#define _HAKONIWA_DEVICE_RUN_H_

#include "athrill_exdev.h"
#include "protobuf/hakoniwa_packet.h"

extern void ex_device_init(MpuAddressRegionType *region, AthrillExDevOperationType *athrill_ops);
extern void ex_device_supply_clock(DeviceClockType *dev_clock);

typedef struct {
	HakoniwaCoreDeviceType		core_device;
	HakoniwaAssetDeviceType		asset_device;

    UdpCommConfigType			udp_config;
    UdpCommType					udp_comm;
    uint32						cpu_freq;
    char						*remote_ipaddr;
    char						*local_ipaddr;
    uint16						client_port;
} HakoniwaAssetDeviceControllerType;

extern HakoniwaAssetDeviceControllerType hakoniwa_asset_controller;


#endif
