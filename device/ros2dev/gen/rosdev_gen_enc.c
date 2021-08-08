#include "rosdev_gen.h"

//TODO void rosdev_topic_enc_<type>(RosReqType* reqp, RosDev<type>Type* out)
void rosdev_topic_enc_Int32(RosReqType* reqp, RosDevInt32Type* out)
{
	out->data = *((rosdev_int32*)(reqp->ptr + 0U));
	return;
}

