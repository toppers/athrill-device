#include "rosdev_gen.h"

void rosdev_topic_dec_Int32(RosReqType* reqp, RosDevInt32Type* in)
{
	*((rosdev_int32*)(reqp->ptr + 0U)) = in->data;
}