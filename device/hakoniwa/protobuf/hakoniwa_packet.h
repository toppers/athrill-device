#ifndef _HAKONIWA_PACKET_H_
#define _HAKONIWA_PACKET_H_


#ifdef __cplusplus
extern "C" {
#endif

typedef unsigned long long Uint64Type;
typedef unsigned int Uint32Type;

typedef struct {
	Uint32Type	version;
	Uint64Type  hakoniwa_time;
} HakoniwaCoreDeviceType;

typedef struct {
	Uint32Type	version;
	Uint64Type  asset_time;
} HakoniwaAssetDeviceType;

typedef struct {
    char *datap;
    int len;
} HakoniwaPacketBufferType;

extern int hakoniwa_packet_actuator_encode(const HakoniwaAssetDeviceType *in, HakoniwaPacketBufferType *out);
extern void hakoniwa_packet_sensor_decode(const HakoniwaPacketBufferType *in, HakoniwaCoreDeviceType *out);

#ifdef __cplusplus
}
#endif

#endif /* _HAKONIWA_PACKET_H_ */
