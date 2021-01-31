#include "serial_client.h"

int main(int argc, char** argv) 
{
  char buf[128];
  int retlen;
  serial_client_init();
  serial_client_put_data(1, "Hello World");
  serial_client_get_data(1, buf, sizeof(buf), &retlen);
  return 0;
}
