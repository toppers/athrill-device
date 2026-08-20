#ifndef HAKO_EXDEV_RUNTIME_H
#define HAKO_EXDEV_RUNTIME_H

#include <stddef.h>

#include "std_types.h"

typedef struct {
    int (*initialize)(const char *asset_name);
    void (*cleanup)(const char *asset_name);
    int (*is_simulation_mode)(void);
    int (*is_pdu_created)(void);
    uint64 (*get_worldtime)(void);
    void (*notify_simtime)(const char *asset_name, uint64 simtime);
    void (*notify_write_pdu_done)(const char *asset_name);
    int (*is_pdu_sync_mode)(const char *asset_name);
    int (*is_pdu_dirty)(
        const char *asset_name,
        const char *robot_name,
        uint32 channel_id);
    int (*read_pdu)(
        const char *asset_name,
        const char *robot_name,
        uint32 channel_id,
        char *data,
        size_t data_size);
    int (*write_pdu)(
        const char *asset_name,
        const char *robot_name,
        uint32 channel_id,
        const char *data,
        size_t data_size);
    void (*notify_read_pdu_done)(const char *asset_name);
} HakoExdevRuntimeOperations;

const HakoExdevRuntimeOperations *hako_exdev_hakoniwa_runtime(void);

#endif
