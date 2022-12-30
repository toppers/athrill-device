#ifndef _EV3PACKET_H_
#define _EV3PACKET_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "ev3_msgs/pdu_ctype_EV3PduActuator.h"
#include "ev3_msgs/pdu_ctype_EV3PduSensor.h"

extern int ev3packet_actuator_encode(const char *in, Hako_Ev3PduActuator *out);
extern void ev3packet_sensor_decode(const Hako_Ev3PduSensor *in, char *out);

#ifdef __cplusplus
}
#endif

#endif /* _EV3PACKET_H_ */