#include "hako_exdev/hakotime.h"

#include <stdio.h>

#define HAKOTIME_EXDEV_MEMORY_SIZE 1024U

static char hakotime_memory[HAKOTIME_EXDEV_MEMORY_SIZE];
static HakotimeContext hakotime_context;

static void hakotime_exdev_initialize(
    MpuAddressRegionType *region,
    AthrillExDevOperationType *athrill_ops)
{
    (void)region;
    if (hakotime_initialize(
            &hakotime_context,
            athrill_ops,
            hako_exdev_hakoniwa_runtime()) != 0) {
        (void)fprintf(stderr, "ERROR: hakotime initialization failed\n");
    }
}

static void hakotime_exdev_supply_clock(DeviceClockType *device_clock)
{
    hakotime_supply_clock(&hakotime_context, device_clock);
}

static void hakotime_exdev_cleanup(void)
{
    hakotime_cleanup(&hakotime_context);
}

ATHRILL_EXDEV_EXPORT AthrillExDeviceType athrill_ex_device = {
    {
        ATHRILL_EXTERNAL_DEVICE_MAGICNO,
        ATHRILL_EXTERNAL_DEVICE_VERSION,
        (int)sizeof(hakotime_memory)
    },
    hakotime_memory,
    NULL,
    hakotime_exdev_initialize,
    hakotime_exdev_supply_clock,
    hakotime_exdev_cleanup
};
