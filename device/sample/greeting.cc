#include "greeting.h"
#include "grpc/client/cpp/sample_client.h"
#include <iostream>

using namespace std;

void greeting_init(void)
{
    cout << "SAMPLE_DEVICE: init from tmori." << endl;
    sample_client_init();
    return;
}

void greeting(const char* strp)
{
    string str = strp;
    cout << str + "Hello world from tmori." << endl;
    sample_client_request(NULL);
    return;
}