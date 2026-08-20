#include "hako_exdev/runtime.h"

#include "hakoniwa_asset_polling.h"

static int runtime_initialize(const char *asset_name)
{
    if (hakoniwa_asset_init() != 0) {
        return -1;
    }
    return hakoniwa_asset_register_polling(asset_name);
}

static void runtime_cleanup(const char *asset_name)
{
    (void)hakoniwa_asset_unregister(asset_name);
}

static int runtime_is_simulation_mode(void)
{
    return hakoniwa_asset_is_simulation_mode();
}

static int runtime_is_pdu_created(void)
{
    return hakoniwa_asset_is_pdu_created();
}

static uint64 runtime_get_worldtime(void)
{
    return (uint64)hakoniwa_asset_get_worldtime();
}

static void runtime_notify_simtime(
    const char *asset_name,
    uint64 simtime)
{
    hakoniwa_asset_notify_simtime(asset_name, (hako_time_t)simtime);
}

static void runtime_notify_write_pdu_done(const char *asset_name)
{
    hakoniwa_asset_notify_write_pdu_done(asset_name);
}

static int runtime_is_pdu_sync_mode(const char *asset_name)
{
    return hakoniwa_asset_is_pdu_sync_mode(asset_name);
}

static int runtime_is_pdu_dirty(
    const char *asset_name,
    const char *robot_name,
    uint32 channel_id)
{
    return hakoniwa_asset_is_pdu_dirty(
        asset_name, robot_name, (HakoPduChannelIdType)channel_id);
}

static int runtime_read_pdu(
    const char *asset_name,
    const char *robot_name,
    uint32 channel_id,
    char *data,
    size_t data_size)
{
    return hakoniwa_asset_read_pdu(
        asset_name,
        robot_name,
        (HakoPduChannelIdType)channel_id,
        data,
        data_size);
}

static int runtime_write_pdu(
    const char *asset_name,
    const char *robot_name,
    uint32 channel_id,
    const char *data,
    size_t data_size)
{
    return hakoniwa_asset_write_pdu(
        asset_name,
        robot_name,
        (HakoPduChannelIdType)channel_id,
        data,
        data_size);
}

static void runtime_notify_read_pdu_done(const char *asset_name)
{
    hakoniwa_asset_notify_read_pdu_done(asset_name);
}

const HakoExdevRuntimeOperations *hako_exdev_hakoniwa_runtime(void)
{
    static const HakoExdevRuntimeOperations operations = {
        runtime_initialize,
        runtime_cleanup,
        runtime_is_simulation_mode,
        runtime_is_pdu_created,
        runtime_get_worldtime,
        runtime_notify_simtime,
        runtime_notify_write_pdu_done,
        runtime_is_pdu_sync_mode,
        runtime_is_pdu_dirty,
        runtime_read_pdu,
        runtime_write_pdu,
        runtime_notify_read_pdu_done
    };
    return &operations;
}
