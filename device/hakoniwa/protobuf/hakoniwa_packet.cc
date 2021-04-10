#include "hakoniwa_packet.h"
#include "hakoniwa_micon.pb.h"
#include <stdio.h>
#include <stdlib.h>

using namespace hakoniwa;

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



int hakoniwa_packet_actuator_encode(const HakoniwaAssetDeviceType *in, HakoniwaPacketBufferType *out)
{
    //allocate protobuf
	MiconPduActuator packet;

    //convert from raw to protobuf
    packet.mutable_header()->set_name("MiconActuatorPdu");
    packet.mutable_header()->set_version(in->version);
    packet.mutable_header()->set_asset_time(in->asset_time);

    std::string serialized_str;
    bool ret = packet.SerializeToString(&serialized_str);
    ASSERT(ret == true);

    ASSERT(out->len >= serialized_str.length());
    memcpy(out->datap, (char*)serialized_str.c_str(), serialized_str.length());
    return serialized_str.length();
}

void hakoniwa_packet_sensor_decode(const HakoniwaPacketBufferType *in, HakoniwaCoreDeviceType *out)
{
    MiconPduSensor packet;
    std::string packet_data(in->datap, in->len);

    //serialized protobuf to protobuf
    bool ret = packet.ParseFromString(packet_data);
    ASSERT(ret == true);

    out->version = packet.mutable_header()->version();
    //printf("version=0x%x\n", packet.mutable_header()->version());
    out->hakoniwa_time = packet.mutable_header()->hakoniwa_time();
    return;
}
