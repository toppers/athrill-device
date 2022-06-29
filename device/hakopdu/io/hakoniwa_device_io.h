#ifndef _HAKONIWA_DEVICE_IO_H_
#define _HAKONIWA_DEVICE_IO_H_

#include "athrill_exdev.h"

extern MpuAddressRegionOperationType	ex_device_memory_operation;

extern char* ex_device_hakopdu_asset_name;
extern void ex_device_memory_init(void);
extern void ex_device_memory_supply_clock(void);
extern AthrillExDevOperationType *athrill_ex_devop;

#endif /* _HAKONIWA_DEVICE_IO_H_ */
