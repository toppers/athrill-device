#include "greeting.h"
#ifdef SAMPLE_ROS2
#include "ros2/workspace/src/hello_world/include/hello_world/sample_client.h"
#else
#include "grpc/client/cpp/sample_client.h"
#endif /* SAMPLE_ROS2 */
#include <iostream>

using namespace std;

void greeting_init(void)
{
    cout << "SAMPLE_DEVICE: init from tmori." << endl;
    sample_client_init();
    return;
}

void greeting(const char* strp, unsigned long long clock)
{
    string str = strp;
    cout << str + "Hello world from tmori." << endl;
    sample_client_request(NULL, clock);
    return;
}