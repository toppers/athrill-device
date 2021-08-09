#include "ros_dev_com.h"
#include "ros_dev.h"

void rosdev_write_data(RosReqType* req)
{
	ros_dev_com_write((rosdev_uint32)req);
}
void rosdev_read_data(RosReqType* req)
{
	ros_dev_com_read((rosdev_uint32)req);
}
