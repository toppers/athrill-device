#include "greeting.h"
#include <iostream>

using namespace std;

void greeting(const char* strp)
{
    string str = strp;
    cout << str + "Hello world from tmori." << endl;
    return;
}