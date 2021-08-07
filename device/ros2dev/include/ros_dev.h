#ifndef _ROS_DEV_H_
#define _ROS_DEV_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

	typedef int RosDevReturnType;
	typedef int RosDevTopicIdType;
	typedef int RosDevSizeType;
	typedef void* RosDevDataPtrType;

	typedef struct {
		RosDevTopicIdType	id;
		RosDevReturnType	ret;
		RosDevSizeType		datalen;
		RosDevDataPtrType	ptr;
	} RosReqType;

	extern void rosdev_write_data(RosReqType* req);
	extern void rosdev_read_data(RosReqType* req);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _ROS_DEV_H_ */