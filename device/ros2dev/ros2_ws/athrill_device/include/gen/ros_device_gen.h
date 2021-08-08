#ifndef _ROS_DEVICE_GEN_H_
#define _ROS_DEVICE_GEN_H_

#include "rosdev_gen_types.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
	//TODO extern RosDevReturnType ros_device_send_<type>(RosDevTopicIdType topic_id, RosDev<type>Type *msg);
	extern RosDevReturnType ros_device_send_Int32(RosDevTopicIdType topic_id, RosDevInt32Type *msg);
	//TODO extern RosDevReturnType ros_device_receive_<type>(RosDevTopicIdType topic_id, RosDev<type>Type *msg);
	extern RosDevReturnType ros_device_receive_Int32(RosDevTopicIdType topic_id, RosDevInt32Type* msg);

	//Turtlebot3
	extern RosDevReturnType ros_device_send_Twist(RosDevTopicIdType topic_id, RosDevTwistType* msg);
	extern RosDevReturnType ros_device_receive_ScanData(RosDevTopicIdType topic_id, RosDevScanDataType* msg);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _ROS_DEVICE_GEN_H_ */