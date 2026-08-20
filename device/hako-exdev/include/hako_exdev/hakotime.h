#ifndef HAKO_EXDEV_HAKOTIME_H
#define HAKO_EXDEV_HAKOTIME_H

#include "athrill_exdev.h"
#include "hako_exdev/runtime.h"

#define HAKOTIME_ASSET_NAME_MAX 255U
#define HAKOTIME_DEFAULT_CPU_FREQ_MHZ 100U

typedef struct {
    const HakoExdevRuntimeOperations *runtime;
    char asset_name[HAKOTIME_ASSET_NAME_MAX + 1U];
    uint32 cpu_freq_mhz;
    uint32 hako_time_only;
    std_bool initialized;
} HakotimeContext;

int hakotime_initialize(
    HakotimeContext *context,
    AthrillExDevOperationType *athrill_ops,
    const HakoExdevRuntimeOperations *runtime);
void hakotime_supply_clock(
    HakotimeContext *context,
    DeviceClockType *device_clock);
void hakotime_cleanup(HakotimeContext *context);

#endif
