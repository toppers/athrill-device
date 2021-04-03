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
    static char read_buffer[1024];

    ssize_t len = read(0, read_buffer, sizeof(read_buffer));
    Ev3RawBufferType out;
    out.datap = buffer;
    out.len = sizeof(buffer);

    Ev3PacketBufferType in;
    in.datap = read_buffer;
    in.len = len;
    printf("len=%ld\n", len);
    ev3packet_sensor_decode((const Ev3PacketBufferType *)&in, &out);

    printf("version=0x%x\n", CONV_UINT32(&buffer[EV3_SENSOR_VERSION_ADDR]));
    printf("simtime=%lu\n", CONV_UINT64(&buffer[EV3_SENSOR_SIMTIME_ADDR]));
    printf("ext_off=%u\n", CONV_UINT32(&buffer[EV3_SENSOR_EXTOFF_ADDR]));
    printf("ext_size=%u\n", CONV_UINT32(&buffer[EV3_SENSOR_EXTSIZE_ADDR]));

    printf("buttons=0x%x\n", *CONV_CHARP(&buffer[EV3_GPIO_BTN_ADDR]));
    printf("color0=%u\n", CONV_UINT32(&buffer[EV3_SENSOR_ADDR_COLOR0]));
    printf("reflect0=%u\n", CONV_UINT32(&buffer[EV3_SENSOR_ADDR_REFLECT0]));
    printf("rgb_r0=%u\n", CONV_UINT32(&buffer[EV3_SENSOR_ADDR_RGB_R0]));
    printf("rgb_g0=%u\n", CONV_UINT32(&buffer[EV3_SENSOR_ADDR_RGB_G0]));
    printf("rgb_b0=%u\n", CONV_UINT32(&buffer[EV3_SENSOR_ADDR_RGB_B0]));

    printf("touch_sensor0=%u\n", CONV_UINT32(&buffer[EV3_SENSOR_ADDR_TOUCH_0]));
    printf("touch_sensor1=%u\n", CONV_UINT32(&buffer[EV3_SENSOR_ADDR_TOUCH_1]));

    printf("gyro_degree=%d\n", CONV_INT32(&buffer[EV3_SENSOR_ADDR_ANGLE]));
    printf("gyro_degree_rate=%d\n", CONV_INT32(&buffer[EV3_SENSOR_ADDR_RATE]));
    printf("sensor_ultrasonic=%u\n", CONV_UINT32(&buffer[EV3_SENSOR_ADDR_ULTRASONIC]));
    printf("motor_angle_a=%u\n", CONV_UINT32(&buffer[EV3_SENSOR_MOTOR_ADDR_ANGLE_A]));
    printf("motor_angle_b=%u\n", CONV_UINT32(&buffer[EV3_SENSOR_MOTOR_ADDR_ANGLE_B]));
    printf("motor_angle_c=%u\n", CONV_UINT32(&buffer[EV3_SENSOR_MOTOR_ADDR_ANGLE_C]));
    printf("gps_lat=%lf\n", CONV_DOUBLE(&buffer[EV3_SENSOR_GPS_ADDR_LAT]));
    printf("gps_lon=%lf\n", CONV_DOUBLE(&buffer[EV3_SENSOR_GPS_ADDR_LON]));
    return 0;
}