#include "hako_exdev/hakotime.h"

#include <stdio.h>
#include <string.h>

#define HAKOTIME_CONFIG_CPU_FREQ "DEVICE_CONFIG_CPU_FREQ_MZ"
#define HAKOTIME_CONFIG_ONLY "DEBUG_FUNC_DEVICE_HAKOTIME_ONLY"
#define HAKOTIME_CONFIG_ASSET_NAME "DEBUG_FUNC_HAKO_ASSET_NAME"

static int hakotime_load_configuration(
    HakotimeContext *context,
    AthrillExDevOperationType *athrill_ops)
{
    char *asset_name = NULL;

    context->cpu_freq_mhz = HAKOTIME_DEFAULT_CPU_FREQ_MHZ;
    context->hako_time_only = 0U;
    if ((athrill_ops == NULL)
        || (athrill_ops->param.get_devcfg_string == NULL)) {
        return -1;
    }
    if (athrill_ops->param.get_devcfg_value != NULL) {
        (void)athrill_ops->param.get_devcfg_value(
            HAKOTIME_CONFIG_CPU_FREQ, &context->cpu_freq_mhz);
        (void)athrill_ops->param.get_devcfg_value(
            HAKOTIME_CONFIG_ONLY, &context->hako_time_only);
    }
    if ((athrill_ops->param.get_devcfg_string(
            HAKOTIME_CONFIG_ASSET_NAME, &asset_name) != STD_E_OK)
        || (asset_name == NULL)
        || (asset_name[0] == '\0')
        || (context->cpu_freq_mhz == 0U)) {
        return -1;
    }
    (void)snprintf(
        context->asset_name,
        sizeof(context->asset_name),
        "%s",
        asset_name);
    return 0;
}

int hakotime_initialize(
    HakotimeContext *context,
    AthrillExDevOperationType *athrill_ops,
    const HakoExdevRuntimeOperations *runtime)
{
    if ((context == NULL)
        || (runtime == NULL)
        || (runtime->initialize == NULL)
        || (runtime->get_worldtime == NULL)
        || (runtime->notify_simtime == NULL)
        || (runtime->is_simulation_mode == NULL)
        || (runtime->is_pdu_created == NULL)) {
        return -1;
    }

    (void)memset(context, 0, sizeof(*context));
    context->runtime = runtime;
    if (hakotime_load_configuration(context, athrill_ops) != 0) {
        return -1;
    }
    if (runtime->initialize(context->asset_name) != 0) {
        return -1;
    }
    context->initialized = TRUE;
    return 0;
}

void hakotime_supply_clock(
    HakotimeContext *context,
    DeviceClockType *device_clock)
{
    uint64 worldtime_ticks;
    uint64 interval_ticks;
    uint64 simulation_time;

    if ((context == NULL)
        || (device_clock == NULL)
        || (context->initialized == FALSE)) {
        return;
    }

    if (context->runtime->is_simulation_mode() == 0) {
        if ((context->hako_time_only != 0U)
            && (context->runtime->notify_write_pdu_done != NULL)) {
            context->runtime->notify_write_pdu_done(context->asset_name);
        }
        device_clock->min_intr_interval = 1U;
    }
    else if (context->runtime->is_pdu_created() == 0) {
        device_clock->min_intr_interval = 1U;
    }
    else {
        worldtime_ticks =
            context->runtime->get_worldtime()
            * (uint64)context->cpu_freq_mhz;
        if ((worldtime_ticks == 0U)
            || (worldtime_ticks <= device_clock->clock)) {
            device_clock->min_intr_interval = 1U;
        }
        else if (device_clock->min_intr_interval
            != (uint64)DEVICE_CLOCK_MAX_INTERVAL) {
            interval_ticks = worldtime_ticks - device_clock->clock;
            if (interval_ticks < device_clock->min_intr_interval) {
                device_clock->min_intr_interval = interval_ticks;
            }
        }
    }

    simulation_time =
        device_clock->clock / (uint64)context->cpu_freq_mhz;
    context->runtime->notify_simtime(
        context->asset_name, simulation_time);
}

void hakotime_cleanup(HakotimeContext *context)
{
    if ((context == NULL) || (context->initialized == FALSE)) {
        return;
    }
    if (context->runtime->cleanup != NULL) {
        context->runtime->cleanup(context->asset_name);
    }
    context->initialized = FALSE;
}
