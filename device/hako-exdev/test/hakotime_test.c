#include "hako_exdev/hakotime.h"

#include <stdio.h>
#include <string.h>

typedef struct {
    int initialize_count;
    int cleanup_count;
    int simulation_mode;
    int pdu_created;
    uint64 worldtime;
    uint64 notified_simtime;
    int write_done_count;
} RuntimeState;

static RuntimeState runtime_state;
static char test_asset_name[] = "athrill-test";

static int mock_initialize(const char *asset_name)
{
    if (strcmp(asset_name, test_asset_name) != 0) {
        return -1;
    }
    runtime_state.initialize_count++;
    return 0;
}

static void mock_cleanup(const char *asset_name)
{
    if (strcmp(asset_name, test_asset_name) == 0) {
        runtime_state.cleanup_count++;
    }
}

static int mock_is_simulation_mode(void)
{
    return runtime_state.simulation_mode;
}

static int mock_is_pdu_created(void)
{
    return runtime_state.pdu_created;
}

static uint64 mock_get_worldtime(void)
{
    return runtime_state.worldtime;
}

static void mock_notify_simtime(const char *asset_name, uint64 simtime)
{
    if (strcmp(asset_name, test_asset_name) == 0) {
        runtime_state.notified_simtime = simtime;
    }
}

static void mock_notify_write_done(const char *asset_name)
{
    if (strcmp(asset_name, test_asset_name) == 0) {
        runtime_state.write_done_count++;
    }
}

static Std_ReturnType mock_get_value(const char *key, uint32 *value)
{
    if (strcmp(key, "DEVICE_CONFIG_CPU_FREQ_MZ") == 0) {
        *value = 100U;
        return STD_E_OK;
    }
    if (strcmp(key, "DEBUG_FUNC_DEVICE_HAKOTIME_ONLY") == 0) {
        *value = 1U;
        return STD_E_OK;
    }
    return STD_E_NOENT;
}

static Std_ReturnType mock_get_string(const char *key, char **value)
{
    if (strcmp(key, "DEBUG_FUNC_HAKO_ASSET_NAME") != 0) {
        return STD_E_NOENT;
    }
    *value = test_asset_name;
    return STD_E_OK;
}

static int expect(int condition, const char *message)
{
    if (!condition) {
        (void)fprintf(stderr, "FAILED: %s\n", message);
        return 1;
    }
    return 0;
}

int main(void)
{
    HakotimeContext context;
    AthrillExDevOperationType athrill_ops;
    HakoExdevRuntimeOperations runtime_ops = {
        .initialize = mock_initialize,
        .cleanup = mock_cleanup,
        .is_simulation_mode = mock_is_simulation_mode,
        .is_pdu_created = mock_is_pdu_created,
        .get_worldtime = mock_get_worldtime,
        .notify_simtime = mock_notify_simtime,
        .notify_write_pdu_done = mock_notify_write_done
    };
    DeviceClockType device_clock;

    (void)memset(&runtime_state, 0, sizeof(runtime_state));
    (void)memset(&athrill_ops, 0, sizeof(athrill_ops));
    athrill_ops.param.get_devcfg_value = mock_get_value;
    athrill_ops.param.get_devcfg_string = mock_get_string;

    if (expect(
            hakotime_initialize(&context, &athrill_ops, &runtime_ops) == 0,
            "initialization")
        || expect(runtime_state.initialize_count == 1, "runtime initialization")
        || expect(context.cpu_freq_mhz == 100U, "CPU frequency")
        || expect(strcmp(context.asset_name, test_asset_name) == 0, "asset name")) {
        return 1;
    }

    (void)memset(&device_clock, 0, sizeof(device_clock));
    runtime_state.simulation_mode = 1;
    runtime_state.pdu_created = 1;
    runtime_state.worldtime = 25U;
    device_clock.clock = 2000U;
    device_clock.min_intr_interval = 1000U;
    hakotime_supply_clock(&context, &device_clock);
    if (expect(device_clock.min_intr_interval == 500U, "worldtime interval")
        || expect(runtime_state.notified_simtime == 20U, "simtime notification")) {
        return 1;
    }

    runtime_state.pdu_created = 0;
    device_clock.min_intr_interval = 1000U;
    hakotime_supply_clock(&context, &device_clock);
    if (expect(device_clock.min_intr_interval == 1U, "non-blocking PDU wait")) {
        return 1;
    }

    runtime_state.simulation_mode = 0;
    device_clock.min_intr_interval = 1000U;
    hakotime_supply_clock(&context, &device_clock);
    if (expect(runtime_state.write_done_count == 1, "time-only notification")
        || expect(device_clock.min_intr_interval == 1U, "inactive simulation")) {
        return 1;
    }

    hakotime_cleanup(&context);
    if (expect(runtime_state.cleanup_count == 1, "runtime cleanup")
        || expect(context.initialized == FALSE, "context cleanup")) {
        return 1;
    }
    return 0;
}
