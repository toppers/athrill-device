#!/bin/bash

TARGET_OS=`uname`

if [ $# -ne 1 ]
then
	echo "$0 {all|clean}"
	exit 1
fi

if [ $1 = "all" ]
then
	cmake .. -DSAMPLE_ROS2=true
	make
	if [ $TARGET_OS = "Darwin" ]
	then
		cp libdevsample.dylib libdevsample.so
	fi
elif [ $1 = "clean" ]
then
	echo "clean up"
	rm -r ./CMakeFiles
	rm Makefile
	rm -r ./ros2
	rm -r ./ament_*
	rm CMake*
	rm cmake*
	rm libdev*
else
	echo "$0 {all|clean}"
	exit 1
fi
