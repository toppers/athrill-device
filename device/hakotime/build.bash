#!/bin/bash

TARGET_OS=`uname`

if [ $# -ne 0 -a $# -ne 1 ]
then
	echo "$0 [clean]"
	exit 1
fi

if [ -d hakoniwa-core-cpp-client ]
then
	:
else
	ln -s ../../third-party/hakoniwa-core-cpp-client .
fi

cd cmake-build
if [ $# -eq 0 ]
then
	cmake ..
	make
elif [ $1 = "clean" ]
then
	echo "clean up"
	rm -rf ./*
else
	echo "$0 [clean]"
	exit 1
fi

