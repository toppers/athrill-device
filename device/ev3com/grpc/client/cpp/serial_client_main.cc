#include "serial_client.h"
#include <string.h>

int main(int argc, char** argv) 
{
  char buf[128];
  int retlen;
  serial_client_init("localhost:50051");
  serial_client_put_data(1, "Hello World", strlen("Hello World") + 1);
  serial_client_get_data(1, buf, sizeof(buf), &retlen);
  return 0;
}
