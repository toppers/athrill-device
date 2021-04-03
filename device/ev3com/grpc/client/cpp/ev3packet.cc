#include "ev3packet.h"
#include "hakoniwa_ev3.pb.h"
#include "ev3com_format.h"

#include <stdio.h>
#include <stdlib.h>

#define ASSERT(expr)	\
do {	\
	if (!(expr))	{	\
		printf("ASSERTION FAILED:%s:%s:%d:%s\n", __FILE__, __FUNCTION__, __LINE__, #expr);	\
		exit(1);	\
	}	\
} while (0)

#define CONV_DOUBLE(p)          (*((double*)(p)))
#define CONV_INT32(p)          (*((google::protobuf::int32*)(p)))
#define CONV_UINT32(p)          (*((google::protobuf::uint32*)(p)))
#define CONV_UINT64(p)          (*((google::protobuf::uint64*)(p)))
#define CONV_CHARP(p)           ((char*)(p))

using namespace hakoniwa;

int ev3packet_actuator_encode(const Ev3RawBufferType *in, Ev3PacketBufferType *out)
{
    //allocate protobuf
    Ev3PduActuator packet;

    //convert from raw to protobuf
    packet.mutable_header()->set_name("ETTX");
    packet.mutable_header()->set_version(CONV_UINT32(&in->datap[EV3_ACTUATOR_VERSION_ADDR]));
    packet.mutable_header()->set_asset_time(CONV_UINT64(&in->datap[EV3_ACTUATOR_SIMTIME_ADDR]));
    packet.mutable_header()->set_ext_off(CONV_UINT32(&in->datap[EV3_ACTUATOR_EXTOFF_ADDR]));
    packet.mutable_header()->set_ext_size(CONV_UINT32(&in->datap[EV3_ACTUATOR_EXTSIZE_ADDR]));

    packet.mutable_body()->set_leds(CONV_CHARP(&in->datap[EV3_GPIO_LED_ADDR]), 1U);

    Ev3PduActuator_Body_Motor *motor = packet.mutable_body()->add_motors();
    ASSERT(motor != NULL);
    motor->set_power(CONV_INT32(&in->datap[EV3_MOTOR_ADDR_POWER_A]));
    motor->set_stop(CONV_UINT32(&in->datap[EV3_MOTOR_ADDR_STOP_A]));
    motor->set_reset_angle(CONV_UINT32(&in->datap[EV3_MOTOR_ADDR_RESET_ANGLE_A]));
    motor = packet.mutable_body()->add_motors();
    ASSERT(motor != NULL);
    motor->set_power(CONV_INT32(&in->datap[EV3_MOTOR_ADDR_POWER_B]));
    motor->set_stop(CONV_UINT32(&in->datap[EV3_MOTOR_ADDR_STOP_B]));
    motor->set_reset_angle(CONV_UINT32(&in->datap[EV3_MOTOR_ADDR_RESET_ANGLE_B]));
    motor = packet.mutable_body()->add_motors();
    ASSERT(motor != NULL);
    motor->set_power(CONV_INT32(&in->datap[EV3_MOTOR_ADDR_POWER_C]));
    motor->set_stop(CONV_UINT32(&in->datap[EV3_MOTOR_ADDR_STOP_C]));
    motor->set_reset_angle(CONV_UINT32(&in->datap[EV3_MOTOR_ADDR_RESET_ANGLE_C]));

    packet.mutable_body()->set_gyro_reset(CONV_UINT32(&in->datap[EV3_GYRO_ADDR_RESET]));

    std::string serialized_str;
    bool ret = packet.SerializeToString(&serialized_str);
    ASSERT(ret == true);

    ASSERT(out->len >= serialized_str.length());
    memcpy(out->datap, (char*)serialized_str.c_str(), serialized_str.length());
    return serialized_str.length();
}

void ev3packet_sensor_decode(const Ev3PacketBufferType *in, Ev3RawBufferType *out)
{
    Ev3PduSensor packet;
    std::string packet_data(in->datap, in->len);

    //serialized protobuf to protobuf
    bool ret = packet.ParseFromString(packet_data);
    ASSERT(ret == true);

    //copy protobuf to raw
    for (int i = 0; i < 4U; i++) {
        *CONV_CHARP(&out->datap[EV3_SENSOR_HEADER_ADDR + i]) = packet.mutable_header()->name().c_str()[i];
        //printf("%c\n", out->datap[EV3_SENSOR_HEADER_ADDR+i]);
    }
    CONV_UINT32(&out->datap[EV3_SENSOR_VERSION_ADDR]) = packet.mutable_header()->version();
    //printf("version=0x%x\n", packet.mutable_header()->version());
    CONV_UINT64(&out->datap[EV3_SENSOR_SIMTIME_ADDR]) = packet.mutable_header()->hakoniwa_time();
    //printf("hakoniwa_time=%ld\n", packet.mutable_header()->hakoniwa_time());
    CONV_UINT32(&out->datap[EV3_SENSOR_EXTOFF_ADDR]) = packet.mutable_header()->ext_off();
    //printf("ext_off=0x%x\n", packet.mutable_header()->ext_off());
    CONV_UINT32(&out->datap[EV3_SENSOR_EXTSIZE_ADDR]) = packet.mutable_header()->ext_size();
    //printf("ext_size=0x%x\n", packet.mutable_header()->ext_size());

    memcpy(CONV_CHARP(&out[EV3_GPIO_BTN_ADDR]), packet.mutable_body()->buttons().c_str(), 1U);
    CONV_UINT32(&out->datap[EV3_SENSOR_ADDR_AMBIENT]) =  0;
    CONV_UINT32(&out->datap[EV3_SENSOR_ADDR_COLOR0]) =  packet.mutable_body()->color_sensors(0).color();
    CONV_UINT32(&out->datap[EV3_SENSOR_ADDR_REFLECT0]) =  packet.mutable_body()->color_sensors(0).reflect();
    CONV_UINT32(&out->datap[EV3_SENSOR_ADDR_RGB_R0]) =  packet.mutable_body()->color_sensors(0).rgb_r();
    CONV_UINT32(&out->datap[EV3_SENSOR_ADDR_RGB_G0]) =  packet.mutable_body()->color_sensors(0).rgb_g();
    CONV_UINT32(&out->datap[EV3_SENSOR_ADDR_RGB_B0]) =  packet.mutable_body()->color_sensors(0).rgb_b();

    CONV_UINT32(&out->datap[EV3_SENSOR_ADDR_COLOR1]) =  packet.mutable_body()->color_sensors(1).color();
    CONV_UINT32(&out->datap[EV3_SENSOR_ADDR_REFLECT1]) =  packet.mutable_body()->color_sensors(1).reflect();
    CONV_UINT32(&out->datap[EV3_SENSOR_ADDR_RGB_R1]) =  packet.mutable_body()->color_sensors(1).rgb_r();
    CONV_UINT32(&out->datap[EV3_SENSOR_ADDR_RGB_G1]) =  packet.mutable_body()->color_sensors(1).rgb_g();
    CONV_UINT32(&out->datap[EV3_SENSOR_ADDR_RGB_B1]) =  packet.mutable_body()->color_sensors(1).rgb_b();

    CONV_UINT32(&out->datap[EV3_SENSOR_ADDR_TOUCH_0]) =  packet.mutable_body()->touch_sensors(0).value();
    CONV_UINT32(&out->datap[EV3_SENSOR_ADDR_TOUCH_1]) =  packet.mutable_body()->touch_sensors(1).value();

    CONV_INT32(&out->datap[EV3_SENSOR_ADDR_ANGLE]) =  packet.mutable_body()->gyro_degree();
    CONV_INT32(&out->datap[EV3_SENSOR_ADDR_RATE]) =  packet.mutable_body()->gyro_degree_rate();

    CONV_UINT32(&out->datap[EV3_SENSOR_ADDR_ULTRASONIC]) =  packet.mutable_body()->sensor_ultrasonic();
    CONV_UINT32(&out->datap[EV3_SENSOR_MOTOR_ADDR_ANGLE_A]) =  packet.mutable_body()->motor_angle_a();
    CONV_UINT32(&out->datap[EV3_SENSOR_MOTOR_ADDR_ANGLE_B]) =  packet.mutable_body()->motor_angle_b();
    CONV_UINT32(&out->datap[EV3_SENSOR_MOTOR_ADDR_ANGLE_C]) =  packet.mutable_body()->motor_angle_c();

    CONV_DOUBLE(&out->datap[EV3_SENSOR_GPS_ADDR_LAT]) =  packet.mutable_body()->gps_lat();
    CONV_DOUBLE(&out->datap[EV3_SENSOR_GPS_ADDR_LON]) =  packet.mutable_body()->gps_lon();
    return;
}

