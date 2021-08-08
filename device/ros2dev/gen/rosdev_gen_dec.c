#include "rosdev_gen.h"

//TODO void rosdev_topic_dec_<type>(RosReqType* reqp, RosDev<type>Type* in)
void rosdev_topic_dec_Int32(RosReqType* reqp, RosDevInt32Type* in)
{
	*((rosdev_int32*)(reqp->ptr + 0U)) = in->data;
}

//TurtleBot3
void rosdev_topic_dec_ScanData(RosReqType* reqp, RosDevScanDataType* in)
{
	int i;
	for (i = 0; i < 360; i++) {
		*((rosdev_float64*)(reqp->ptr + (i * sizeof(rosdev_float64)) )) = in->ranges[i];
	}
}