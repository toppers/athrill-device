#include "rosdev_gen.h"
#include <stdio.h>
#include <stddef.h>

#include "ros_device_gen.h"

static void rosdev_topic_pub_func_id_0(RosReqType* reqp);
static void rosdev_topic_sub_func_id_0(RosReqType* reqp);


void (*rosdev_topic_pub_func[ROSDEV_TOPIC_PUB_ID_NUM]) (RosReqType* reqp) = {
	rosdev_topic_pub_func_id_0,
};

void (*rosdev_topic_sub_func[ROSDEV_TOPIC_SUB_ID_NUM]) (RosReqType * reqp) = {
	rosdev_topic_sub_func_id_0,
};

static void rosdev_topic_pub_func_id_0(RosReqType* reqp)
{
	RosDevInt32Type msg;
	rosdev_topic_enc_Int32(reqp, &msg);

	//publish
	reqp->ret = ros_device_send_Int32(reqp->id, &msg);

	return;
}
static void rosdev_topic_sub_func_id_0(RosReqType* reqp)
{
	RosDevInt32Type msg;

	//receive
	reqp->ret = ros_device_receive_Int32(reqp->id, &msg);

	//if data received
	if (reqp->ret == ROSDEV_E_OK) {
		rosdev_topic_dec_Int32(reqp, &msg);
	}
	return;
}
