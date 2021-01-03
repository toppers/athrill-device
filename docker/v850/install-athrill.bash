#!/bin/bash

cd athrill-target-v850e2m/build_linux
make clean; rm -f *.o;
make timer32=true serial_fifo_enable=true
