#include "ros2dev_gen.h"

void ros2dev_topic_dec_Int32(RosReqType* reqp, Ros2DevInt32Type* in)
{
	*((rosdev_int32*)(reqp->ptr + 0U)) = in->data;
}