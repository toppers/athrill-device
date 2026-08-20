#ifndef HAKO_EXDEV_HAKOPDU_EV3_H
#define HAKO_EXDEV_HAKOPDU_EV3_H

#include "hako_exdev/hakotime.h"

#define HAKOPDU_EV3_ASSET_NAME_SIZE 256U
#define HAKOPDU_EV3_ROBOT_NAME_SIZE 256U
#define HAKOPDU_EV3_RX_DATA_SIZE 0x1000U
#define HAKOPDU_EV3_TX_DATA_SIZE 0x1000U
#define HAKOPDU_EV3_RX_PDU_SIZE 248U
#define HAKOPDU_EV3_TX_PDU_SIZE 196U

#define HAKOPDU_EV3_BASE 0x090F0000U
#define HAKOPDU_EV3_RX_BASE HAKOPDU_EV3_BASE
#define HAKOPDU_EV3_TX_BASE (HAKOPDU_EV3_BASE + 0x1000U)
#define HAKOPDU_EV3_TX_FLAG_BASE (HAKOPDU_EV3_BASE + 0x2000U)

typedef struct {
    HakotimeContext time;
    const HakoExdevRuntimeOperations *runtime;
    char asset_name[HAKOPDU_EV3_ASSET_NAME_SIZE];
    char robot_name[HAKOPDU_EV3_ROBOT_NAME_SIZE];
    uint32 tx_channel_id;
    uint32 rx_channel_id;
    uint8 tx_data[HAKOPDU_EV3_TX_DATA_SIZE];
    uint8 rx_data[HAKOPDU_EV3_RX_DATA_SIZE];
    uint8 tx_pdu[HAKOPDU_EV3_TX_PDU_SIZE];
    uint8 rx_pdu[HAKOPDU_EV3_RX_PDU_SIZE];
    std_bool tx_dirty;
    std_bool initialized;
} HakopduEv3Context;

int hakopdu_ev3_initialize(
    HakopduEv3Context *context,
    AthrillExDevOperationType *athrill_ops,
    const HakoExdevRuntimeOperations *runtime);
void hakopdu_ev3_supply_clock(
    HakopduEv3Context *context,
    DeviceClockType *device_clock);
void hakopdu_ev3_cleanup(HakopduEv3Context *context);

Std_ReturnType hakopdu_ev3_get_data(
    HakopduEv3Context *context,
    uint32 address,
    void *data,
    size_t size);
Std_ReturnType hakopdu_ev3_put_data(
    HakopduEv3Context *context,
    uint32 address,
    const void *data,
    size_t size);
Std_ReturnType hakopdu_ev3_get_pointer(
    HakopduEv3Context *context,
    uint32 address,
    uint8 **data);

#endif
