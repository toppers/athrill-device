#ifndef _ROS_DEV_PRIMITIVE_TYPES_H_
#define _ROS_DEV_PRIMITIVE_TYPES_H_

typedef signed char rosdev_int8;
typedef signed short rosdev_int16;
typedef signed int rosdev_int32;
typedef signed long long rosdev_int64;

typedef unsigned char rosdev_uint8;
typedef unsigned short rosdev_uint16;
typedef unsigned int rosdev_uint32;
typedef unsigned long long rosdev_uint64;
typedef int rosdev_bool;
typedef float rosdev_float32;
typedef double rosdev_float64;

typedef rosdev_int32 RosDevReturnType;
typedef rosdev_int32 RosDevTopicIdType;
typedef rosdev_int32 RosDevSizeType;
typedef void* RosDevDataPtrType;

#define ROSDEV_E_OK			0U
#define ROSDEV_E_NOENT		2U
#define ROSDEV_E_INVALID	13U


#endif /* _ROS_DEV_PRIMITIVE_TYPES_H_ */