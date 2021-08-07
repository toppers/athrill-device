#ifndef _ROS2DEV_GEN_H_
#define _ROS2DEV_GEN_H_

#include "ros_dev.h"
#include "ros2dev_gen_types.h"

#define ROS2DEV_TOPIC_ID_NUM	1U

extern void (*ros2dev_topic_pub_func[ROS2DEV_TOPIC_ID_NUM]) (RosReqType *reqp);
extern void (*ros2dev_topic_sub_func[ROS2DEV_TOPIC_ID_NUM]) (RosReqType* reqp);

extern void ros2dev_topic_enc_Int32(RosReqType* reqp, Ros2DevInt32Type* out);
extern void ros2dev_topic_dec_Int32(RosReqType* reqp, Ros2DevInt32Type* in);

#endif /* _ROS2DEV_GEN_H_ */