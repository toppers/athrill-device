#include "serial_client.h"

int main(int argc, char** argv) 
{
  serial_client_init();
  serial_client_put_data(1, "Hello World");
  return 0;
}
