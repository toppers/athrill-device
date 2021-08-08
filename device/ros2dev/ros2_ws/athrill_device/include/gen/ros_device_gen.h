#ifndef _ROS_DEVICE_GEN_H_
#define _ROS_DEVICE_GEN_H_

#include "rosdev_gen_types.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

	extern RosDevReturnType ros_device_send_Int32(RosDevTopicIdType topic_id, RosDevInt32Type *msg);
	extern RosDevReturnType ros_device_receive_Int32(RosDevTopicIdType topic_id, RosDevInt32Type* msg);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _ROS_DEVICE_GEN_H_ */