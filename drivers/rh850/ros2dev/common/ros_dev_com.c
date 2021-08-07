#include "ros_dev_com.h"
#include "ros_dev.h"

void rosdev_write_data(RosReqType* req)
{
	ros_dev_com_write((uint32)req);
}
void rosdev_read_data(RosReqType* req)
{
	ros_dev_com_read((uint32)req);
}
