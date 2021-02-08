#include "serial_client.h"
#include <string.h>
#include <stdio.h>

int main(int argc, char** argv) 
{
  char buf[128];
  int retlen;

  serial_client_init("172.25.0.1:50051");
  while (1) {
    serial_client_get_data(1, buf, sizeof(buf), &retlen);
    printf("RECV DATA: %s\n",buf);
    printf("SEND DATA: %s\n",buf);
    serial_client_put_data(1, buf, strlen(buf));
  }
  return 0;
}
