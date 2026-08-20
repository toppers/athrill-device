#include "hako_exdev/hakopdu_ev3.h"

#include <stdio.h>

#define HAKOPDU_EV3_EXDEV_MEMORY_SIZE (1024U * 1024U)

static char hakopdu_ev3_memory[HAKOPDU_EV3_EXDEV_MEMORY_SIZE];
static HakopduEv3Context hakopdu_ev3_context;

static Std_ReturnType get_data8(
    MpuAddressRegionType *region, CoreIdType core_id, uint32 address, uint8 *data)
{
    (void)region;
    (void)core_id;
    return hakopdu_ev3_get_data(&hakopdu_ev3_context, address, data, sizeof(*data));
}

static Std_ReturnType get_data16(
    MpuAddressRegionType *region, CoreIdType core_id, uint32 address, uint16 *data)
{
    (void)region;
    (void)core_id;
    return hakopdu_ev3_get_data(&hakopdu_ev3_context, address, data, sizeof(*data));
}

static Std_ReturnType get_data32(
    MpuAddressRegionType *region, CoreIdType core_id, uint32 address, uint32 *data)
{
    (void)region;
    (void)core_id;
    return hakopdu_ev3_get_data(&hakopdu_ev3_context, address, data, sizeof(*data));
}

static Std_ReturnType put_data8(
    MpuAddressRegionType *region, CoreIdType core_id, uint32 address, uint8 data)
{
    (void)region;
    (void)core_id;
    return hakopdu_ev3_put_data(&hakopdu_ev3_context, address, &data, sizeof(data));
}

static Std_ReturnType put_data16(
    MpuAddressRegionType *region, CoreIdType core_id, uint32 address, uint16 data)
{
    (void)region;
    (void)core_id;
    return hakopdu_ev3_put_data(&hakopdu_ev3_context, address, &data, sizeof(data));
}

static Std_ReturnType put_data32(
    MpuAddressRegionType *region, CoreIdType core_id, uint32 address, uint32 data)
{
    (void)region;
    (void)core_id;
    return hakopdu_ev3_put_data(&hakopdu_ev3_context, address, &data, sizeof(data));
}

static Std_ReturnType get_pointer(
    MpuAddressRegionType *region, CoreIdType core_id, uint32 address, uint8 **data)
{
    (void)region;
    (void)core_id;
    return hakopdu_ev3_get_pointer(&hakopdu_ev3_context, address, data);
}

static MpuAddressRegionOperationType hakopdu_ev3_memory_operations = {
    get_data8,
    get_data16,
    get_data32,
    put_data8,
    put_data16,
    put_data32,
    get_pointer
};

static void hakopdu_ev3_exdev_initialize(
    MpuAddressRegionType *region,
    AthrillExDevOperationType *athrill_ops)
{
    (void)region;
    if (hakopdu_ev3_initialize(
            &hakopdu_ev3_context,
            athrill_ops,
            hako_exdev_hakoniwa_runtime()) != 0) {
        (void)fprintf(stderr, "ERROR: hakopdu_ev3 initialization failed\n");
    }
}

static void hakopdu_ev3_exdev_supply_clock(DeviceClockType *device_clock)
{
    hakopdu_ev3_supply_clock(&hakopdu_ev3_context, device_clock);
}

static void hakopdu_ev3_exdev_cleanup(void)
{
    hakopdu_ev3_cleanup(&hakopdu_ev3_context);
}

ATHRILL_EXDEV_EXPORT AthrillExDeviceType athrill_ex_device = {
    {
        ATHRILL_EXTERNAL_DEVICE_MAGICNO,
        ATHRILL_EXTERNAL_DEVICE_VERSION,
        (int)sizeof(hakopdu_ev3_memory)
    },
    hakopdu_ev3_memory,
    &hakopdu_ev3_memory_operations,
    hakopdu_ev3_exdev_initialize,
    hakopdu_ev3_exdev_supply_clock,
    hakopdu_ev3_exdev_cleanup
};
