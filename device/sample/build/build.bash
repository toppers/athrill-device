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
elif [ $1 = "clean" ]
then
	echo "clean up"
	rm -r ./CMakeFiles
	rm Makefile
	#rm sample_client*
	#rm sample.*
	rm CMake*
	rm cmake*
	rm libdev*
	rm -r ./grpc
else
	echo "$0 {all|clean}"
	exit 1
fi
