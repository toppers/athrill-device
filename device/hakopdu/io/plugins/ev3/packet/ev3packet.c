#include "ev3packet.h"
#include "ev3com_format.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define ASSERT(expr)	\
do {	\
	if (!(expr))	{	\
		printf("ASSERTION FAILED:%s:%s:%d:%s\n", __FILE__, __FUNCTION__, __LINE__, #expr);	\
		exit(1);	\
	}	\
} while (0)

#define CONV_DOUBLE(p)          (*((double*)(p)))
#define CONV_INT32(p)           (*((Hako_int32*)(p)))
#define CONV_UINT32(p)          (*((Hako_uint32*)(p)))
#define CONV_UINT64(p)          (*((Hako_uint64*)(p)))
#define CONV_CHARP(p)           ((char*)(p))


int ev3packet_actuator_encode(const char *in, Hako_Ev3PduActuator *out)
{
    //convert from raw to protobuf
    //packet.mutable_header()->set_name("ETTX");
    memcpy(out->head.name, "ETTX", 4);
    out->head.name[4] = '\0';

    //packet.mutable_header()->set_version(CONV_UINT32(&in->datap[EV3_ACTUATOR_VERSION_ADDR]));
    out->head.version = CONV_UINT32(&in[EV3_ACTUATOR_VERSION_ADDR]);
    //packet.mutable_header()->set_asset_time(CONV_UINT64(&in->datap[EV3_ACTUATOR_SIMTIME_ADDR]));
    out->head.asset_time = CONV_UINT64(&in[EV3_ACTUATOR_SIMTIME_ADDR]);
    //packet.mutable_header()->set_ext_off(CONV_UINT32(&in->datap[EV3_ACTUATOR_EXTOFF_ADDR]));
    out->head.ext_off = CONV_UINT32(&in[EV3_ACTUATOR_EXTOFF_ADDR]);
    //packet.mutable_header()->set_ext_size(CONV_UINT32(&in->datap[EV3_ACTUATOR_EXTSIZE_ADDR]));
    out->head.ext_size = CONV_UINT32(&in[EV3_ACTUATOR_EXTSIZE_ADDR]);

    //packet.mutable_body()->set_leds(CONV_CHARP(&in->datap[EV3_GPIO_LED_ADDR]), 1U);
    out->leds[0] = in[EV3_GPIO_LED_ADDR];

    //Ev3PduActuator_Body_Motor *motor = packet.mutable_body()->add_motors();
    //ASSERT(motor != NULL);
    //motor->set_power(CONV_INT32(&in->datap[EV3_MOTOR_ADDR_POWER_A]));
    //motor->set_stop(CONV_UINT32(&in->datap[EV3_MOTOR_ADDR_STOP_A]));
    //motor->set_reset_angle(CONV_UINT32(&in->datap[EV3_MOTOR_ADDR_RESET_ANGLE_A]));
    out->motors[0].power = CONV_INT32(&in[EV3_MOTOR_ADDR_POWER_A]);
    out->motors[0].stop = CONV_UINT32(&in[EV3_MOTOR_ADDR_STOP_A]);
    out->motors[0].reset_angle = CONV_UINT32(&in[EV3_MOTOR_ADDR_RESET_ANGLE_A]);
    //motor = packet.mutable_body()->add_motors();
    //ASSERT(motor != NULL);
    //motor->set_power(CONV_INT32(&in->datap[EV3_MOTOR_ADDR_POWER_B]));
    //motor->set_stop(CONV_UINT32(&in->datap[EV3_MOTOR_ADDR_STOP_B]));
    //motor->set_reset_angle(CONV_UINT32(&in->datap[EV3_MOTOR_ADDR_RESET_ANGLE_B]));
    out->motors[1].power = CONV_INT32(&in[EV3_MOTOR_ADDR_POWER_B]);
    out->motors[1].stop = CONV_UINT32(&in[EV3_MOTOR_ADDR_STOP_B]);
    out->motors[1].reset_angle = CONV_UINT32(&in[EV3_MOTOR_ADDR_RESET_ANGLE_B]);
    //motor = packet.mutable_body()->add_motors();
    //ASSERT(motor != NULL);
    //motor->set_power(CONV_INT32(&in->datap[EV3_MOTOR_ADDR_POWER_C]));
    //motor->set_stop(CONV_UINT32(&in->datap[EV3_MOTOR_ADDR_STOP_C]));
    //motor->set_reset_angle(CONV_UINT32(&in->datap[EV3_MOTOR_ADDR_RESET_ANGLE_C]));
    out->motors[2].power = CONV_INT32(&in[EV3_MOTOR_ADDR_POWER_C]);
    out->motors[2].stop = CONV_UINT32(&in[EV3_MOTOR_ADDR_STOP_C]);
    out->motors[2].reset_angle = CONV_UINT32(&in[EV3_MOTOR_ADDR_RESET_ANGLE_C]);

    //packet.mutable_body()->set_gyro_reset(CONV_UINT32(&in->datap[EV3_GYRO_ADDR_RESET]));
    out->gyro_reset = CONV_UINT32(&in[EV3_GYRO_ADDR_RESET]);

    //std::string serialized_str;
    //bool ret = packet.SerializeToString(&serialized_str);
    //ASSERT(ret == true);

    //ASSERT(out->len >= serialized_str.length());
    //memcpy(out->datap, (char*)serialized_str.c_str(), serialized_str.length());
    //return serialized_str.length();
    return 0;
}

void ev3packet_sensor_decode(const Hako_Ev3PduSensor *in, char *out)
{
    //Ev3PduSensor packet;
    //std::string packet_data(in->datap, in->len);

    //serialized protobuf to protobuf
    //bool ret = packet.ParseFromString(packet_data);
    //ASSERT(ret == true);

    //copy protobuf to raw
    for (int i = 0; i < 4U; i++) {
        out[EV3_SENSOR_HEADER_ADDR + i] = in->head.name[i];
        //printf("%c\n", out->datap[EV3_SENSOR_HEADER_ADDR+i]);
    }
    //CONV_UINT32(&out->datap[EV3_SENSOR_VERSION_ADDR]) = packet.mutable_header()->version();
    //printf("version=0x%x\n", packet.mutable_header()->version());
    CONV_UINT32(&out[EV3_SENSOR_VERSION_ADDR]) = in->head.version;

    //CONV_UINT64(&out->datap[EV3_SENSOR_SIMTIME_ADDR]) = packet.mutable_header()->hakoniwa_time();
    //printf("hakoniwa_time=%ld\n", packet.mutable_header()->hakoniwa_time());
    CONV_UINT64(&out[EV3_SENSOR_SIMTIME_ADDR]) = in->head.hakoniwa_time;

    //CONV_UINT32(&out->datap[EV3_SENSOR_EXTOFF_ADDR]) = packet.mutable_header()->ext_off();
    //printf("ext_off=0x%x\n", packet.mutable_header()->ext_off());
    CONV_UINT32(&out[EV3_SENSOR_EXTOFF_ADDR]) = in->head.ext_off;

    //CONV_UINT32(&out->datap[EV3_SENSOR_EXTSIZE_ADDR]) = packet.mutable_header()->ext_size();
    //printf("ext_size=0x%x\n", packet.mutable_header()->ext_size());
    CONV_UINT32(&out[EV3_SENSOR_EXTSIZE_ADDR]) = in->head.ext_size;

    //memcpy(CONV_CHARP(&out->datap[EV3_GPIO_BTN_ADDR]), packet.mutable_body()->buttons().c_str(), 1U);
    memcpy(&out[EV3_GPIO_BTN_ADDR], in->buttons, 1);

    //CONV_UINT32(&out->datap[EV3_SENSOR_ADDR_AMBIENT]) =  0;
    CONV_UINT32(&out[EV3_SENSOR_ADDR_AMBIENT]) = 0;

    //CONV_UINT32(&out->datap[EV3_SENSOR_ADDR_COLOR0]) =  packet.mutable_body()->color_sensors(0).color();
    //CONV_UINT32(&out->datap[EV3_SENSOR_ADDR_REFLECT0]) =  packet.mutable_body()->color_sensors(0).reflect();
    //CONV_UINT32(&out->datap[EV3_SENSOR_ADDR_RGB_R0]) =  packet.mutable_body()->color_sensors(0).rgb_r();
    //CONV_UINT32(&out->datap[EV3_SENSOR_ADDR_RGB_G0]) =  packet.mutable_body()->color_sensors(0).rgb_g();
    //CONV_UINT32(&out->datap[EV3_SENSOR_ADDR_RGB_B0]) =  packet.mutable_body()->color_sensors(0).rgb_b();
    CONV_UINT32(&out[EV3_SENSOR_ADDR_COLOR0]) = in->color_sensors[0].color;
    CONV_UINT32(&out[EV3_SENSOR_ADDR_REFLECT0]) = in->color_sensors[0].reflect;
    CONV_UINT32(&out[EV3_SENSOR_ADDR_RGB_R0]) = in->color_sensors[0].rgb_r;
    CONV_UINT32(&out[EV3_SENSOR_ADDR_RGB_G0]) = in->color_sensors[0].rgb_g;
    CONV_UINT32(&out[EV3_SENSOR_ADDR_RGB_B0]) = in->color_sensors[0].rgb_b;

    //CONV_UINT32(&out->datap[EV3_SENSOR_ADDR_COLOR1]) =  packet.mutable_body()->color_sensors(1).color();
    //CONV_UINT32(&out->datap[EV3_SENSOR_ADDR_REFLECT1]) =  packet.mutable_body()->color_sensors(1).reflect();
    //CONV_UINT32(&out->datap[EV3_SENSOR_ADDR_RGB_R1]) =  packet.mutable_body()->color_sensors(1).rgb_r();
    //CONV_UINT32(&out->datap[EV3_SENSOR_ADDR_RGB_G1]) =  packet.mutable_body()->color_sensors(1).rgb_g();
    //CONV_UINT32(&out->datap[EV3_SENSOR_ADDR_RGB_B1]) =  packet.mutable_body()->color_sensors(1).rgb_b();
    CONV_UINT32(&out[EV3_SENSOR_ADDR_COLOR1]) = in->color_sensors[1].color;
    CONV_UINT32(&out[EV3_SENSOR_ADDR_REFLECT1]) = in->color_sensors[1].reflect;
    CONV_UINT32(&out[EV3_SENSOR_ADDR_RGB_R1]) = in->color_sensors[1].rgb_r;
    CONV_UINT32(&out[EV3_SENSOR_ADDR_RGB_G1]) = in->color_sensors[1].rgb_g;
    CONV_UINT32(&out[EV3_SENSOR_ADDR_RGB_B1]) = in->color_sensors[1].rgb_b;

    //CONV_UINT32(&out->datap[EV3_SENSOR_ADDR_TOUCH_0]) =  packet.mutable_body()->touch_sensors(0).value();
    //CONV_UINT32(&out->datap[EV3_SENSOR_ADDR_TOUCH_1]) =  packet.mutable_body()->touch_sensors(1).value();
    CONV_UINT32(&out[EV3_SENSOR_ADDR_TOUCH_0]) = in->touch_sensors[0].value;
    CONV_UINT32(&out[EV3_SENSOR_ADDR_TOUCH_1]) = in->touch_sensors[1].value;

    //CONV_INT32(&out->datap[EV3_SENSOR_ADDR_ANGLE]) =  packet.mutable_body()->gyro_degree();
    //CONV_INT32(&out->datap[EV3_SENSOR_ADDR_RATE]) =  packet.mutable_body()->gyro_degree_rate();
    CONV_INT32(&out[EV3_SENSOR_ADDR_ANGLE]) = in->gyro_degree;
    CONV_INT32(&out[EV3_SENSOR_ADDR_RATE]) = in->gyro_degree_rate;

    //CONV_UINT32(&out->datap[EV3_SENSOR_ADDR_ULTRASONIC]) =  packet.mutable_body()->sensor_ultrasonic();
    //CONV_UINT32(&out->datap[EV3_SENSOR_MOTOR_ADDR_ANGLE_A]) =  packet.mutable_body()->motor_angle_a();
    //CONV_UINT32(&out->datap[EV3_SENSOR_MOTOR_ADDR_ANGLE_B]) =  packet.mutable_body()->motor_angle_b();
    //CONV_UINT32(&out->datap[EV3_SENSOR_MOTOR_ADDR_ANGLE_C]) =  packet.mutable_body()->motor_angle_c();
    CONV_UINT32(&out[EV3_SENSOR_ADDR_ULTRASONIC]) = in->sensor_ultrasonic;
    CONV_UINT32(&out[EV3_SENSOR_MOTOR_ADDR_ANGLE_A]) = in->motor_angle[0];
    CONV_UINT32(&out[EV3_SENSOR_MOTOR_ADDR_ANGLE_B]) = in->motor_angle[1];
    CONV_UINT32(&out[EV3_SENSOR_MOTOR_ADDR_ANGLE_C]) = in->motor_angle[2];

    //CONV_DOUBLE(&out->datap[EV3_SENSOR_GPS_ADDR_LAT]) =  packet.mutable_body()->gps_lat();
    //CONV_DOUBLE(&out->datap[EV3_SENSOR_GPS_ADDR_LON]) =  packet.mutable_body()->gps_lon();
    CONV_DOUBLE(&out[EV3_SENSOR_GPS_ADDR_LAT]) = in->gps_lat;
    CONV_DOUBLE(&out[EV3_SENSOR_GPS_ADDR_LON]) = in->gps_lon;
    return;
}

