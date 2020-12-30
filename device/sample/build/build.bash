#!/bin/bash

TARGET_OS=`uname`
OPENSSL_PATH=/usr/local/opt/openssl

if [ $# -ne 1 ]
then
	echo "$0 {all|clean}"
	exit 1
fi

if [ $1 = "all" ]
then
	if [ $TARGET_OS = "Linux" ]
	then
		cmake .. -DProtobuf_DIR=/usr/local/grpc/lib/cmake/protobuf  -Wno-dev
	else
		cmake .. -DOPENSSL_ROOT_DIR=${OPENSSL_PATH}
	fi
	make
else
	rm -rf ./CMakeFiles
	rm -f Makefile
	rm -f sample_client*
	rm -f sample.*
	rm -f CMake*
	rm -f cmake*
	rm -f libdev*
	rm -rf ./grpc
fi
