#include <stdio.h>
#include "ev3packet.h"
#include <unistd.h>
#include "hakoniwa_ev3.pb.h"
#include "ev3com_format.h"

static char buffer[1024];
#define CONV_DOUBLE(p)          (*((double*)(p)))
#define CONV_INT32(p)          (*((google::protobuf::int32*)(p)))
#define CONV_UINT32(p)          (*((google::protobuf::uint32*)(p)))
#define CONV_UINT64(p)          (*((google::protobuf::uint64*)(p)))
#define CONV_CHARP(p)           ((char*)(p))

int main(int argc, const char* argv[])
{
    static char tx_memory[1024];
    CONV_UINT32(&buffer[EV3_ACTUATOR_VERSION_ADDR]) = 1;
    CONV_UINT64(&buffer[EV3_ACTUATOR_SIMTIME_ADDR]) = 1024;
    CONV_UINT32(&buffer[EV3_ACTUATOR_EXTOFF_ADDR]) = 512;
    CONV_UINT32(&buffer[EV3_ACTUATOR_EXTSIZE_ADDR]) = 512;

    *CONV_CHARP(&buffer[EV3_GPIO_LED_ADDR]) = 0xFF;
    CONV_UINT32(&buffer[EV3_MOTOR_ADDR_POWER_A]) = 10;
    CONV_UINT32(&buffer[EV3_MOTOR_ADDR_STOP_A]) = 11;
    CONV_UINT32(&buffer[EV3_MOTOR_ADDR_RESET_ANGLE_A]) = 12;
    CONV_UINT32(&buffer[EV3_MOTOR_ADDR_POWER_B]) = 13;
    CONV_UINT32(&buffer[EV3_MOTOR_ADDR_STOP_B]) = 14;
    CONV_UINT32(&buffer[EV3_MOTOR_ADDR_RESET_ANGLE_B]) = 15;
    CONV_UINT32(&buffer[EV3_MOTOR_ADDR_POWER_C]) = 16;
    CONV_UINT32(&buffer[EV3_MOTOR_ADDR_STOP_C]) = 17;
    CONV_UINT32(&buffer[EV3_MOTOR_ADDR_RESET_ANGLE_C]) = 18;

    CONV_UINT32(&buffer[EV3_GYRO_ADDR_RESET]) = 99;
    Ev3RawBufferType raw;
    raw.datap = buffer;
    raw.len = sizeof(buffer);

    Ev3PacketBufferType packet;
    packet.datap = tx_memory;
    packet.len = sizeof(tx_memory);
    int len = ev3packet_actuator_encode((const Ev3RawBufferType *)&raw, &packet);

    write(1, packet.datap, len);

    return 0;
}