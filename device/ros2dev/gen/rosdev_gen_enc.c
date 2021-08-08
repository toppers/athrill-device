#include "rosdev_gen.h"

void rosdev_topic_enc_Int32(RosReqType* reqp, RosDevInt32Type* out)
{
	out->data = *((rosdev_int32*)(reqp->ptr + 0U));
	return;
}

