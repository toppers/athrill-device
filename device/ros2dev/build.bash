#!/bin/bash

CDIR=`pwd`
if [ "${CDIR}" = "/root/workspace/athrill-device/device/ros2dev/workspace_build" ]
then
	:
else
	echo "ERROR: please cd directory to /root/workspace/athrill-device/device/ros2dev/workspace_build"
	exit 1
fi

if [ $# -ne 1 ]
then
	echo "Usage: $0 {all|clean}"
	exit 1
fi

OPT=${1}
if [ "$OPT" = "all" ]
then
	cmake ..
	make
else
	rm -rf ./*
fi
