#ifndef _ROS_DEV_COM_H_
#define _ROS_DEV_COM_H_

#include "ros_dev_reg.h"
#include "ros_dev_primitive_types.h"
#include "Os.h"
#include "t_syslog.h"
#include "t_stdlib.h"
#include "sysmod/serial.h"
#include "sysmod/syslog.h"
#include "prc_sil.h"

static inline void ros_dev_com_write(rosdev_uint32 ptr)
{
	sil_wrw_mem((void*)ROS_DEV_REG_PUB_ADDR, ptr);
}
static inline void ros_dev_com_read(rosdev_uint32 ptr)
{
	sil_wrw_mem((void*)ROS_DEV_REG_SUB_ADDR, ptr);
}

#endif /* _ROS_DEV_COM_H_ */