#ifndef _HAKONIWA_DEVICE_IO_H_
#define _HAKONIWA_DEVICE_IO_H_

#include "athrill_exdev.h"

#define EX_DEVICE_MEMORY_START  0x090F0000
#define EX_DEVICE_MEMORY_SIZE	(1U * 1024U) /* Bytes */

#define HAKO_SMAPLEDEV_TX_ADDR EX_DEVICE_MEMORY_START
#define HAKO_SMAPLEDEV_RX_ADDR (EX_DEVICE_MEMORY_START + 4U)

#define HAKO_SAMPLEDEV_ASSET_NAME       "athrill"
#define HAKO_SAMPLEDEV_ROBO_NAME        "Robot"
#define HAKO_SAMPLEDEV_CHANNEL_TX_ID    0
#define HAKO_SAMPLEDEV_CHANNEL_TX_SIZE  4
#define HAKO_SAMPLEDEV_CHANNEL_RX_ID    1
#define HAKO_SAMPLEDEV_CHANNEL_RX_SIZE  4

extern MpuAddressRegionOperationType	ex_device_memory_operation;

#endif /* _HAKONIWA_DEVICE_IO_H_ */
