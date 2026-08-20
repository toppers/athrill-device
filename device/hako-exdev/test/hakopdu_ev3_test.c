#include "hako_exdev/hakopdu_ev3.h"

#include <stdio.h>
#include <string.h>

typedef struct {
    int simulation_mode;
    int pdu_created;
    int pdu_sync_mode;
    int pdu_dirty;
    uint64 worldtime;
    uint8 rx_pdu[HAKOPDU_EV3_RX_PDU_SIZE];
    uint8 tx_pdu[HAKOPDU_EV3_TX_PDU_SIZE];
    size_t written_size;
    unsigned int read_done_count;
    unsigned int write_done_count;
    unsigned int cleanup_count;
} MockState;

static MockState state;

static int expect(int condition, const char *message)
{
    if (!condition) {
        (void)fprintf(stderr, "FAILED: %s\n", message);
        return 1;
    }
    return 0;
}

static void store(uint8 *data, size_t offset, const void *value, size_t size)
{
    (void)memcpy(&data[offset], value, size);
}

static void load(const uint8 *data, size_t offset, void *value, size_t size)
{
    (void)memcpy(value, &data[offset], size);
}

static Std_ReturnType mock_get_value(const char *key, unsigned int *value)
{
    if (strcmp(key, "DEVICE_CONFIG_CPU_FREQ_MZ") == 0) {
        *value = 100U;
    }
    else if (strcmp(key, "DEBUG_FUNC_DEVICE_HAKOTIME_ONLY") == 0) {
        *value = 0U;
    }
    else if (strcmp(key, "DEBUG_FUNC_EV3COM_CHANNEL_ID_TX") == 0) {
        *value = 1U;
    }
    else if (strcmp(key, "DEBUG_FUNC_EV3COM_CHANNEL_ID_RX") == 0) {
        *value = 2U;
    }
    else {
        return STD_E_NOENT;
    }
    return STD_E_OK;
}

static Std_ReturnType mock_get_string(const char *key, char **value)
{
    static char asset_name[] = "athrill";
    static char robot_name[] = "Robot";

    if (strcmp(key, "DEBUG_FUNC_HAKO_ASSET_NAME") == 0) {
        *value = asset_name;
    }
    else if (strcmp(key, "DEBUG_FUNC_HAKO_ROBO_NAME") == 0) {
        *value = robot_name;
    }
    else {
        return STD_E_NOENT;
    }
    return STD_E_OK;
}

static int mock_initialize(const char *asset_name)
{
    return strcmp(asset_name, "athrill");
}

static void mock_cleanup(const char *asset_name)
{
    (void)asset_name;
    state.cleanup_count++;
}

static int mock_is_simulation_mode(void) { return state.simulation_mode; }
static int mock_is_pdu_created(void) { return state.pdu_created; }
static uint64 mock_get_worldtime(void) { return state.worldtime; }
static void mock_notify_simtime(const char *name, uint64 time)
{
    (void)name;
    (void)time;
}
static void mock_notify_write_done(const char *name)
{
    (void)name;
    state.write_done_count++;
}
static int mock_is_pdu_sync_mode(const char *name)
{
    (void)name;
    return state.pdu_sync_mode;
}
static int mock_is_pdu_dirty(
    const char *asset, const char *robot, uint32 channel)
{
    if ((strcmp(asset, "athrill") != 0)
        || (strcmp(robot, "Robot") != 0) || (channel != 2U)) {
        return 0;
    }
    return state.pdu_dirty;
}
static int mock_read_pdu(
    const char *asset, const char *robot, uint32 channel,
    char *data, size_t size)
{
    (void)asset;
    (void)robot;
    if ((channel != 2U) || (size != sizeof(state.rx_pdu))) {
        return -1;
    }
    (void)memcpy(data, state.rx_pdu, size);
    return 0;
}
static int mock_write_pdu(
    const char *asset, const char *robot, uint32 channel,
    const char *data, size_t size)
{
    (void)asset;
    (void)robot;
    if ((channel != 1U) || (size != sizeof(state.tx_pdu))) {
        return -1;
    }
    (void)memcpy(state.tx_pdu, data, size);
    state.written_size = size;
    return 0;
}
static void mock_notify_read_done(const char *name)
{
    (void)name;
    state.read_done_count++;
}

int main(void)
{
    HakopduEv3Context context;
    AthrillExDevOperationType athrill_ops;
    DeviceClockType clock;
    const HakoExdevRuntimeOperations runtime = {
        .initialize = mock_initialize,
        .cleanup = mock_cleanup,
        .is_simulation_mode = mock_is_simulation_mode,
        .is_pdu_created = mock_is_pdu_created,
        .get_worldtime = mock_get_worldtime,
        .notify_simtime = mock_notify_simtime,
        .notify_write_pdu_done = mock_notify_write_done,
        .is_pdu_sync_mode = mock_is_pdu_sync_mode,
        .is_pdu_dirty = mock_is_pdu_dirty,
        .read_pdu = mock_read_pdu,
        .write_pdu = mock_write_pdu,
        .notify_read_pdu_done = mock_notify_read_done
    };
    const uint32 sensor_color = 6U;
    const sint32 motor_power = -42;
    uint32 actual_color = 0U;
    sint32 actual_power = 0;
    uint8 led = 3U;
    uint8 actual_button = 0U;

    (void)memset(&state, 0, sizeof(state));
    (void)memset(&athrill_ops, 0, sizeof(athrill_ops));
    athrill_ops.param.get_devcfg_value = mock_get_value;
    athrill_ops.param.get_devcfg_string = mock_get_string;
    if (expect(hakopdu_ev3_initialize(
            &context, &athrill_ops, &runtime) == 0, "initialize")) {
        return 1;
    }

    (void)memcpy(state.rx_pdu, "ETRX", 4U);
    state.rx_pdu[152U] = 0x15U;
    store(state.rx_pdu, 156U, &sensor_color, sizeof(sensor_color));
    state.simulation_mode = 1;
    state.pdu_created = 1;
    state.pdu_dirty = 1;
    state.worldtime = 10U;
    (void)hakopdu_ev3_put_data(
        &context, HAKOPDU_EV3_TX_BASE, &led, sizeof(led));
    (void)hakopdu_ev3_put_data(
        &context, HAKOPDU_EV3_TX_BASE + 4U,
        &motor_power, sizeof(motor_power));
    clock.clock = 500U;
    clock.min_intr_interval = 1000U;
    hakopdu_ev3_supply_clock(&context, &clock);

    (void)hakopdu_ev3_get_data(
        &context, HAKOPDU_EV3_RX_BASE, &actual_button, sizeof(actual_button));
    (void)hakopdu_ev3_get_data(
        &context, HAKOPDU_EV3_RX_BASE + 8U,
        &actual_color, sizeof(actual_color));
    load(state.tx_pdu, 156U, &actual_power, sizeof(actual_power));
    if (expect(clock.min_intr_interval == 500U, "embedded hakotime")
        || expect(state.read_done_count == 1U, "read notification")
        || expect(state.written_size == HAKOPDU_EV3_TX_PDU_SIZE, "TX size")
        || expect(memcmp(state.tx_pdu, "ETTX", 4U) == 0, "TX header")
        || expect(state.tx_pdu[152U] == led, "LED encoding")
        || expect(actual_power == motor_power, "motor encoding")
        || expect(actual_button == 0x15U, "button decoding")
        || expect(actual_color == sensor_color, "color decoding")) {
        return 2;
    }

    state.simulation_mode = 0;
    state.pdu_sync_mode = 1;
    state.written_size = 0U;
    clock.min_intr_interval = 1000U;
    hakopdu_ev3_supply_clock(&context, &clock);
    if (expect(state.write_done_count == 1U, "sync write notification")
        || expect(context.tx_dirty == FALSE, "sync clears dirty")
        || expect(clock.min_intr_interval == 1U, "inactive clock")) {
        return 3;
    }

    if (expect(hakopdu_ev3_get_data(
            &context, HAKOPDU_EV3_TX_FLAG_BASE + 1U,
            &actual_button, sizeof(actual_button)) == STD_E_SEGV,
            "out-of-range access")) {
        return 4;
    }
    hakopdu_ev3_cleanup(&context);
    return expect(state.cleanup_count == 1U, "cleanup");
}
