#ifndef _ROSDEV_GEN_TYPES_H_
#define _ROSDEV_GEN_TYPES_H_

#include "ros_dev_primitive_types.h"

//TODO
typedef struct {
	rosdev_int32 data;
} RosDevInt32Type;

//TurtleBot3
typedef struct {
	rosdev_float64 ranges[360];
} RosDevScanDataType;
typedef struct {
	rosdev_float64 x;
	rosdev_float64 y;
	rosdev_float64 z;
} RosDevVector3Type;
typedef struct {
	RosDevVector3Type linear;
	RosDevVector3Type angular;
} RosDevTwistType;

#endif /* _ROSDEV_GEN_TYPES_H_ */