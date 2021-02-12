#include "serial_client.h"
#include <string.h>
#include <stdio.h>
#include <unistd.h>

int main(int argc, char** argv) 
{
  char buf[128];
  int retlen;
  char ip_port[128];

  if (argc != 3) {
    printf("Usage: %s <ipaddr> <portno>\n", argv[0]);
    return 1;
  }
  sprintf(ip_port, "%s:%s", argv[1], argv[2]);

  serial_client_init(ip_port);
  while (1) {
    serial_client_get_data(1, buf, sizeof(buf), &retlen);
    printf("RECV DATA: %s",buf);
    printf("SEND DATA: %s",buf);
    serial_client_put_data(1, buf, strlen(buf));

    usleep(1000*1000);
  }
  return 0;
}
