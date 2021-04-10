#!/bin/bash

TARGET_OS=`uname`
OPENSSL_PATH=/usr/local/opt/openssl

if [ $# -ne 0 -a $# -ne 1 ]
then
	echo "$0 [clean]"
	exit 1
fi

if [ $# -eq 0 ]
then
	if [ $TARGET_OS = "Linux" ] || [ $TARGET_OS = "Darwin" ]
	then
		cmake .. -DProtobuf_DIR=/usr/local/grpc/lib/cmake/protobuf  -Wno-dev
	else
		cmake .. -DOPENSSL_ROOT_DIR=${OPENSSL_PATH}
	fi
	make
elif [ $1 = "clean" ]
then
	echo "clean up"
	rm -r ./CMakeFiles
	rm -r protobuf
	rm Makefile
	rm CMake*
	rm cmake*
	rm *.so
else
	echo "$0 [clean]"
	exit 1
fi
