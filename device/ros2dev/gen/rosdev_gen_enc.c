#include "rosdev_gen.h"

//TODO void rosdev_topic_enc_<type>(RosReqType* reqp, RosDev<type>Type* out)
void rosdev_topic_enc_Int32(RosReqType* reqp, RosDevInt32Type* out)
{
	out->data = *((rosdev_int32*)(reqp->ptr + 0U));
	return;
}

//Turtlebot3
void rosdev_topic_enc_Twist(RosReqType* reqp, RosDevTwistType* out)
{
	out->linear.x = *((rosdev_float64*)(reqp->ptr + (0U * sizeof(rosdev_float64))));
	out->linear.y = *((rosdev_float64*)(reqp->ptr + (1U * sizeof(rosdev_float64))));
	out->linear.z = *((rosdev_float64*)(reqp->ptr + (2U * sizeof(rosdev_float64))));
	out->angular.x = *((rosdev_float64*)(reqp->ptr + (3U * sizeof(rosdev_float64))));
	out->angular.y = *((rosdev_float64*)(reqp->ptr + (4U * sizeof(rosdev_float64))));
	out->angular.z = *((rosdev_float64*)(reqp->ptr + (5U * sizeof(rosdev_float64))));
	return;
}


