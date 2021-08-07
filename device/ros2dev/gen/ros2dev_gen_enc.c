#include "ros2dev_gen.h"

void ros2dev_topic_enc_Int32(RosReqType* reqp, Ros2DevInt32Type* out)
{
	out->data = *((rosdev_int32*)(reqp->ptr + 0U));
	return;
}

