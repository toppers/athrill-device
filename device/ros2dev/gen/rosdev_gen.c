#include "rosdev_gen.h"
#include <stdio.h>
#include <stddef.h>

#include "ros_device_gen.h"
//TODO rosdev_topic_pub_func_id_<ID>
static void rosdev_topic_pub_func_id_0(RosReqType* reqp);
//TODO rosdev_topic_sub_func_id_<ID>
static void rosdev_topic_sub_func_id_0(RosReqType* reqp);

//Turtlebot3
static void rosdev_topic_pub_func_id_1(RosReqType* reqp);
static void rosdev_topic_sub_func_id_1(RosReqType* reqp);

void (*rosdev_topic_pub_func[ROSDEV_TOPIC_PUB_ID_NUM]) (RosReqType* reqp) = {
	//TODO rosdev_topic_pub_func_id_<ID>,
	rosdev_topic_pub_func_id_0,
	rosdev_topic_pub_func_id_1,
};

void (*rosdev_topic_sub_func[ROSDEV_TOPIC_SUB_ID_NUM]) (RosReqType * reqp) = {
	//TODO rosdev_topic_sub_func_id_<ID>,
	rosdev_topic_sub_func_id_0,
	rosdev_topic_sub_func_id_1,
};

//TODO rosdev_topic_pub_func_id_<ID>,
static void rosdev_topic_pub_func_id_0(RosReqType* reqp)
{
	//TODO RosDev<type>Type msg;,
	RosDevInt32Type msg;
	//TODO rosdev_topic_enc_<type>(reqp, &msg);
	rosdev_topic_enc_Int32(reqp, &msg);

	//publish
	//TODO reqp->ret = ros_device_send_<type>(reqp->id, &msg);
	reqp->ret = ros_device_send_Int32(reqp->id, &msg);

	return;
}
//TODO rosdev_topic_sub_func_id_<ID>,
static void rosdev_topic_sub_func_id_0(RosReqType* reqp)
{
	//TODO RosDev<type>Type msg;,
	RosDevInt32Type msg;

	//receive
	//TODO reqp->ret = ros_device_receive_<type>(reqp->id, &msg);
	reqp->ret = ros_device_receive_Int32(reqp->id, &msg);

	//if data received
	if (reqp->ret == ROSDEV_E_OK) {
		//TODO rosdev_topic_dec_<type>(reqp, &msg);
		rosdev_topic_dec_Int32(reqp, &msg);
	}
	return;
}

//Turtlebot3
static void rosdev_topic_pub_func_id_1(RosReqType* reqp)
{
	//printf("rosdev_topic_pub_func_id_1:enter\n");
	//TODO RosDev<type>Type msg;,
	RosDevTwistType msg;
	//TODO rosdev_topic_enc_<type>(reqp, &msg);
	rosdev_topic_enc_Twist(reqp, &msg);

	//publish
	//TODO reqp->ret = ros_device_send_<type>(reqp->id, &msg);
	reqp->ret = ros_device_send_Twist(reqp->id, &msg);

	return;
}
static void rosdev_topic_sub_func_id_1(RosReqType* reqp)
{
	//printf("rosdev_topic_sub_func_id_1:enter\n");
	//TODO RosDev<type>Type msg;,
	RosDevScanDataType msg;

	//receive
	//TODO reqp->ret = ros_device_receive_<type>(reqp->id, &msg);
	reqp->ret = ros_device_receive_ScanData(reqp->id, &msg);

	//if data received
	if (reqp->ret == ROSDEV_E_OK) {
		//TODO rosdev_topic_dec_<type>(reqp, &msg);
		rosdev_topic_dec_ScanData(reqp, &msg);
	}
	return;
}
