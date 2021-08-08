#ifndef _ROSDEV_GEN_H_
#define _ROSDEV_GEN_H_

#include "ros_dev.h"
#include "rosdev_gen_types.h"

//TODO #define ROSDEV_TOPIC_PUB_ID_NUM	<PUB_ID_NUM>
#define ROSDEV_TOPIC_PUB_ID_NUM	1U
//TODO #define ROSDEV_TOPIC_SUB_ID_NUM	<SUB_ID_NUM>
#define ROSDEV_TOPIC_SUB_ID_NUM	1U

extern void (*rosdev_topic_pub_func[ROSDEV_TOPIC_PUB_ID_NUM]) (RosReqType *reqp);
extern void (*rosdev_topic_sub_func[ROSDEV_TOPIC_SUB_ID_NUM]) (RosReqType* reqp);

//TODO extern void rosdev_topic_enc_<type>(RosReqType* reqp, RosDev<type>Type* out);
extern void rosdev_topic_enc_Int32(RosReqType* reqp, RosDevInt32Type* out);
//TODO extern void rosdev_topic_dec_<type>(RosReqType* reqp, RosDev<type>Type* out);
extern void rosdev_topic_dec_Int32(RosReqType* reqp, RosDevInt32Type* in);

#endif /* _ROSDEV_GEN_H_ */