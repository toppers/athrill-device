#include "hako_exdev/hakopdu_ev3.h"

#include <stdio.h>
#include <string.h>

#define HAKOPDU_CONFIG_ROBOT_NAME "DEBUG_FUNC_HAKO_ROBO_NAME"
#define HAKOPDU_CONFIG_TX_CHANNEL "DEBUG_FUNC_EV3COM_CHANNEL_ID_TX"
#define HAKOPDU_CONFIG_RX_CHANNEL "DEBUG_FUNC_EV3COM_CHANNEL_ID_RX"

#define EV3_HEADER_SIZE 32U
#define EV3_HEADER_VERSION_OFFSET 4U
#define EV3_HEADER_TIME_OFFSET 16U
#define EV3_HEADER_EXT_OFFSET 24U
#define EV3_HEADER_EXT_SIZE_OFFSET 28U
#define EV3_TX_TIME_OFFSET 8U
#define EV3_BODY_OFFSET 32U
#define EV3_BUTTON_OFFSET (EV3_BODY_OFFSET + 0U)
#define EV3_SENSOR_OFFSET(index) (EV3_BODY_OFFSET + 4U + ((index) * 4U))
#define EV3_MOTOR_ANGLE_OFFSET(index) (EV3_BODY_OFFSET + 256U + ((index) * 4U))
#define EV3_GPS_LAT_OFFSET (EV3_BODY_OFFSET + 480U)
#define EV3_GPS_LON_OFFSET (EV3_BODY_OFFSET + 488U)
#define EV3_LED_OFFSET (EV3_BODY_OFFSET + 0U)
#define EV3_MOTOR_OFFSET(index) (EV3_BODY_OFFSET + 4U + ((index) * 4U))
#define EV3_GYRO_RESET_OFFSET (EV3_BODY_OFFSET + 52U)

typedef struct {
    char name[128];
    uint32 version;
    sint64 asset_time;
    uint32 ext_off;
    uint32 ext_size;
} Ev3ActuatorHeader;

typedef struct {
    sint32 power;
    uint32 stop;
    uint32 reset_angle;
} Ev3Motor;

typedef struct {
    Ev3ActuatorHeader head;
    uint8 leds[1];
    Ev3Motor motors[3];
    uint32 gyro_reset;
} Ev3ActuatorPdu;

typedef struct {
    char name[128];
    uint32 version;
    sint64 hakoniwa_time;
    uint32 ext_off;
    uint32 ext_size;
} Ev3SensorHeader;

typedef struct {
    uint32 color;
    uint32 reflect;
    uint32 rgb_r;
    uint32 rgb_g;
    uint32 rgb_b;
} Ev3ColorSensor;

typedef struct {
    uint32 value;
} Ev3TouchSensor;

typedef struct {
    Ev3SensorHeader head;
    uint8 buttons[1];
    Ev3ColorSensor color_sensors[2];
    Ev3TouchSensor touch_sensors[2];
    uint32 motor_angle[3];
    sint32 gyro_degree;
    sint32 gyro_degree_rate;
    uint32 sensor_ultrasonic;
    double gps_lat;
    double gps_lon;
} Ev3SensorPdu;

_Static_assert(
    (offsetof(Ev3ActuatorPdu, gyro_reset) + sizeof(uint32))
        == HAKOPDU_EV3_TX_PDU_SIZE,
    "EV3 actuator PDU ABI mismatch");
_Static_assert(sizeof(Ev3SensorPdu) == HAKOPDU_EV3_RX_PDU_SIZE,
    "EV3 sensor PDU ABI mismatch");

static void load_bytes(const uint8 *source, size_t offset, void *value, size_t size)
{
    (void)memcpy(value, &source[offset], size);
}

static void store_bytes(uint8 *destination, size_t offset, const void *value, size_t size)
{
    (void)memcpy(&destination[offset], value, size);
}

static void initialize_tx_data(HakopduEv3Context *context)
{
    static const char header[4] = {'E', 'T', 'T', 'X'};
    const uint32 version = 1U;
    const uint32 ext_offset = 512U;
    const uint32 ext_size = 512U;

    (void)memset(context->tx_data, 0, sizeof(context->tx_data));
    store_bytes(context->tx_data, 0U, header, sizeof(header));
    store_bytes(context->tx_data, EV3_HEADER_VERSION_OFFSET,
        &version, sizeof(version));
    store_bytes(context->tx_data, EV3_HEADER_EXT_OFFSET,
        &ext_offset, sizeof(ext_offset));
    store_bytes(context->tx_data, EV3_HEADER_EXT_SIZE_OFFSET,
        &ext_size, sizeof(ext_size));
}

static void encode_actuator(HakopduEv3Context *context)
{
    Ev3ActuatorPdu pdu;
    uint32 index;

    (void)memset(&pdu, 0, sizeof(pdu));
    (void)memcpy(pdu.head.name, "ETTX", 4U);
    load_bytes(context->tx_data, EV3_HEADER_VERSION_OFFSET,
        &pdu.head.version, sizeof(pdu.head.version));
    load_bytes(context->tx_data, EV3_TX_TIME_OFFSET,
        &pdu.head.asset_time, sizeof(pdu.head.asset_time));
    load_bytes(context->tx_data, EV3_HEADER_EXT_OFFSET,
        &pdu.head.ext_off, sizeof(pdu.head.ext_off));
    load_bytes(context->tx_data, EV3_HEADER_EXT_SIZE_OFFSET,
        &pdu.head.ext_size, sizeof(pdu.head.ext_size));
    pdu.leds[0] = context->tx_data[EV3_LED_OFFSET];
    for (index = 0U; index < 3U; index++) {
        load_bytes(context->tx_data, EV3_MOTOR_OFFSET(index),
            &pdu.motors[index].power, sizeof(pdu.motors[index].power));
        load_bytes(context->tx_data, EV3_MOTOR_OFFSET(index + 4U),
            &pdu.motors[index].stop, sizeof(pdu.motors[index].stop));
        load_bytes(context->tx_data, EV3_MOTOR_OFFSET(index + 8U),
            &pdu.motors[index].reset_angle,
            sizeof(pdu.motors[index].reset_angle));
    }
    load_bytes(context->tx_data, EV3_GYRO_RESET_OFFSET,
        &pdu.gyro_reset, sizeof(pdu.gyro_reset));
    (void)memcpy(context->tx_pdu, &pdu, sizeof(context->tx_pdu));
}

static void decode_sensor(HakopduEv3Context *context)
{
    Ev3SensorPdu pdu;
    const uint32 ambient = 0U;
    uint32 index;

    (void)memcpy(&pdu, context->rx_pdu, sizeof(pdu));
    store_bytes(context->rx_data, 0U, pdu.head.name, 4U);
    store_bytes(context->rx_data, EV3_HEADER_VERSION_OFFSET,
        &pdu.head.version, sizeof(pdu.head.version));
    store_bytes(context->rx_data, EV3_HEADER_TIME_OFFSET,
        &pdu.head.hakoniwa_time, sizeof(pdu.head.hakoniwa_time));
    store_bytes(context->rx_data, EV3_HEADER_EXT_OFFSET,
        &pdu.head.ext_off, sizeof(pdu.head.ext_off));
    store_bytes(context->rx_data, EV3_HEADER_EXT_SIZE_OFFSET,
        &pdu.head.ext_size, sizeof(pdu.head.ext_size));
    context->rx_data[EV3_BUTTON_OFFSET] = pdu.buttons[0];
    store_bytes(context->rx_data, EV3_SENSOR_OFFSET(0U),
        &ambient, sizeof(ambient));
    for (index = 0U; index < 2U; index++) {
        const uint32 base = (index == 0U) ? 1U : 31U;
        store_bytes(context->rx_data, EV3_SENSOR_OFFSET(base),
            &pdu.color_sensors[index].color, sizeof(uint32));
        store_bytes(context->rx_data, EV3_SENSOR_OFFSET(base + 1U),
            &pdu.color_sensors[index].reflect, sizeof(uint32));
        store_bytes(context->rx_data, EV3_SENSOR_OFFSET(base + 2U),
            &pdu.color_sensors[index].rgb_r, sizeof(uint32));
        store_bytes(context->rx_data, EV3_SENSOR_OFFSET(base + 3U),
            &pdu.color_sensors[index].rgb_g, sizeof(uint32));
        store_bytes(context->rx_data, EV3_SENSOR_OFFSET(base + 4U),
            &pdu.color_sensors[index].rgb_b, sizeof(uint32));
    }
    store_bytes(context->rx_data, EV3_SENSOR_OFFSET(27U),
        &pdu.touch_sensors[0].value, sizeof(uint32));
    store_bytes(context->rx_data, EV3_SENSOR_OFFSET(30U),
        &pdu.touch_sensors[1].value, sizeof(uint32));
    store_bytes(context->rx_data, EV3_SENSOR_OFFSET(6U),
        &pdu.gyro_degree, sizeof(pdu.gyro_degree));
    store_bytes(context->rx_data, EV3_SENSOR_OFFSET(7U),
        &pdu.gyro_degree_rate, sizeof(pdu.gyro_degree_rate));
    store_bytes(context->rx_data, EV3_SENSOR_OFFSET(21U),
        &pdu.sensor_ultrasonic, sizeof(pdu.sensor_ultrasonic));
    for (index = 0U; index < 3U; index++) {
        store_bytes(context->rx_data, EV3_MOTOR_ANGLE_OFFSET(index),
            &pdu.motor_angle[index], sizeof(uint32));
    }
    store_bytes(context->rx_data, EV3_GPS_LAT_OFFSET,
        &pdu.gps_lat, sizeof(pdu.gps_lat));
    store_bytes(context->rx_data, EV3_GPS_LON_OFFSET,
        &pdu.gps_lon, sizeof(pdu.gps_lon));
}

static int runtime_has_pdu_operations(const HakoExdevRuntimeOperations *runtime)
{
    return (runtime != NULL)
        && (runtime->is_pdu_sync_mode != NULL)
        && (runtime->is_pdu_dirty != NULL)
        && (runtime->read_pdu != NULL)
        && (runtime->write_pdu != NULL)
        && (runtime->notify_read_pdu_done != NULL);
}

int hakopdu_ev3_initialize(
    HakopduEv3Context *context,
    AthrillExDevOperationType *athrill_ops,
    const HakoExdevRuntimeOperations *runtime)
{
    char *robot_name = NULL;

    if ((context == NULL) || (athrill_ops == NULL)
        || (runtime_has_pdu_operations(runtime) == 0)
        || (athrill_ops->param.get_devcfg_string == NULL)
        || (athrill_ops->param.get_devcfg_value == NULL)) {
        return -1;
    }
    (void)memset(context, 0, sizeof(*context));
    if (hakotime_initialize(&context->time, athrill_ops, runtime) != 0) {
        return -1;
    }
    if ((athrill_ops->param.get_devcfg_string(
            HAKOPDU_CONFIG_ROBOT_NAME, &robot_name) != STD_E_OK)
        || (robot_name == NULL) || (robot_name[0] == '\0')
        || (athrill_ops->param.get_devcfg_value(
            HAKOPDU_CONFIG_TX_CHANNEL, &context->tx_channel_id) != STD_E_OK)
        || (athrill_ops->param.get_devcfg_value(
            HAKOPDU_CONFIG_RX_CHANNEL, &context->rx_channel_id) != STD_E_OK)) {
        hakotime_cleanup(&context->time);
        return -1;
    }
    context->runtime = runtime;
    (void)memcpy(context->asset_name,
        context->time.asset_name, sizeof(context->asset_name));
    (void)snprintf(context->robot_name, sizeof(context->robot_name),
        "%s", robot_name);
    initialize_tx_data(context);
    context->initialized = TRUE;
    return 0;
}

static void supply_pdu(HakopduEv3Context *context)
{
    if (context->runtime->is_pdu_created() == 0) {
        return;
    }
    if (context->runtime->is_simulation_mode() != 0) {
        if ((context->runtime->is_pdu_dirty(
                context->asset_name,
                context->robot_name,
                context->rx_channel_id) != 0)
            && (context->runtime->read_pdu(
                context->asset_name,
                context->robot_name,
                context->rx_channel_id,
                (char *)context->rx_pdu,
                sizeof(context->rx_pdu)) == 0)) {
            decode_sensor(context);
        }
        context->runtime->notify_read_pdu_done(context->asset_name);
        if (context->tx_dirty != FALSE) {
            encode_actuator(context);
            (void)context->runtime->write_pdu(
                context->asset_name,
                context->robot_name,
                context->tx_channel_id,
                (const char *)context->tx_pdu,
                sizeof(context->tx_pdu));
        }
    }
    else if (context->runtime->is_pdu_sync_mode(context->asset_name) != 0) {
        encode_actuator(context);
        (void)context->runtime->write_pdu(
            context->asset_name,
            context->robot_name,
            context->tx_channel_id,
            (const char *)context->tx_pdu,
            sizeof(context->tx_pdu));
        if (context->runtime->notify_write_pdu_done != NULL) {
            context->runtime->notify_write_pdu_done(context->asset_name);
        }
        context->tx_dirty = FALSE;
    }
}

void hakopdu_ev3_supply_clock(
    HakopduEv3Context *context,
    DeviceClockType *device_clock)
{
    if ((context == NULL) || (context->initialized == FALSE)) {
        return;
    }
    supply_pdu(context);
    hakotime_supply_clock(&context->time, device_clock);
}

void hakopdu_ev3_cleanup(HakopduEv3Context *context)
{
    if ((context == NULL) || (context->initialized == FALSE)) {
        return;
    }
    hakotime_cleanup(&context->time);
    context->initialized = FALSE;
}

static Std_ReturnType map_data(
    HakopduEv3Context *context,
    uint32 address,
    size_t size,
    uint8 **data)
{
    uint32 offset;

    if ((context == NULL) || (data == NULL) || (size == 0U)) {
        return STD_E_INVALID;
    }
    if ((address >= HAKOPDU_EV3_RX_BASE)
        && (address < HAKOPDU_EV3_TX_BASE)) {
        offset = (address - HAKOPDU_EV3_RX_BASE) + EV3_HEADER_SIZE;
        if (((size_t)offset + size) <= sizeof(context->rx_data)) {
            *data = &context->rx_data[offset];
            return STD_E_OK;
        }
    }
    else if ((address >= HAKOPDU_EV3_TX_BASE)
        && (address < HAKOPDU_EV3_TX_FLAG_BASE)) {
        offset = (address - HAKOPDU_EV3_TX_BASE) + EV3_HEADER_SIZE;
        if (((size_t)offset + size) <= sizeof(context->tx_data)) {
            *data = &context->tx_data[offset];
            return STD_E_OK;
        }
    }
    return STD_E_SEGV;
}

Std_ReturnType hakopdu_ev3_get_data(
    HakopduEv3Context *context,
    uint32 address,
    void *data,
    size_t size)
{
    uint8 *source;
    Std_ReturnType result;

    if (data == NULL) {
        return STD_E_INVALID;
    }
    result = map_data(context, address, size, &source);
    if (result == STD_E_OK) {
        (void)memcpy(data, source, size);
    }
    return result;
}

Std_ReturnType hakopdu_ev3_put_data(
    HakopduEv3Context *context,
    uint32 address,
    const void *data,
    size_t size)
{
    uint8 *destination;
    Std_ReturnType result;

    if (data == NULL) {
        return STD_E_INVALID;
    }
    if ((address == HAKOPDU_EV3_TX_FLAG_BASE) && (size == 1U)) {
        return STD_E_OK;
    }
    result = map_data(context, address, size, &destination);
    if ((result == STD_E_OK) && (address >= HAKOPDU_EV3_TX_BASE)) {
        (void)memcpy(destination, data, size);
        context->tx_dirty = TRUE;
        return STD_E_OK;
    }
    return (result == STD_E_OK) ? STD_E_PERM : result;
}

Std_ReturnType hakopdu_ev3_get_pointer(
    HakopduEv3Context *context,
    uint32 address,
    uint8 **data)
{
    return map_data(context, address, 1U, data);
}
