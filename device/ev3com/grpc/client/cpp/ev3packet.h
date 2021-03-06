#ifndef _EV3PACKET_H_
#define _EV3PACKET_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    char *datap;
    int len;
} Ev3RawBufferType;

typedef struct {
    char *datap;
    int len;
} Ev3PacketBufferType;

extern int ev3packet_actuator_encode(const Ev3RawBufferType *in, Ev3PacketBufferType *out);
extern void ev3packet_sensor_decode(const Ev3PacketBufferType *in, Ev3RawBufferType *out);

#ifdef __cplusplus
}
#endif

#endif /* _EV3PACKET_H_ */