#include "ros2dev_gen.h"
#include <stdio.h>
#include <stddef.h>

#include "athrill_device_gen.h"

static void ros2dev_topic_pub_func_id_0(RosReqType* reqp);
static void ros2dev_topic_sub_func_id_0(RosReqType* reqp);


void (*ros2dev_topic_pub_func[ROS2DEV_TOPIC_ID_NUM]) (RosReqType* reqp) = {
	ros2dev_topic_pub_func_id_0,
};

void (*ros2dev_topic_sub_func[ROS2DEV_TOPIC_ID_NUM]) (RosReqType * reqp) = {
	ros2dev_topic_sub_func_id_0,
};

static void ros2dev_topic_pub_func_id_0(RosReqType* reqp)
{
	Ros2DevInt32Type msg;
	ros2dev_topic_enc_Int32(reqp, &msg);

	//printf("id=%d\n", reqp->id);
	//printf("ret=%d\n", reqp->ret);
	//printf("datalen=%d\n", reqp->datalen);
	//printf("ptr_addr=%p\n", reqp->ptr);

	//publish
	reqp->ret = athrill_device_send_Int32(reqp->id, &msg);

	printf("value=%d\n", msg.data);
	return;
}
static void ros2dev_topic_sub_func_id_0(RosReqType* reqp)
{
	Ros2DevInt32Type msg;

	//receive
	reqp->ret = athrill_device_receive_Int32(reqp->id, &msg);

	//if data received
	if (reqp->ret == ROSDEV_E_OK) {
		ros2dev_topic_dec_Int32(reqp, &msg);
	}
	return;
}
