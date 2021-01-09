#!/bin/bash

ATHRILL_TOP_DIR=$(cd ../../.. && pwd)
DOCKER_IMAGE=kanetugu2015/ros2-foxy:v1.0.0

sudo docker run \
	-v ${ATHRILL_TOP_DIR}/athrill:/root/workspace/athrill \
	-v ${ATHRILL_TOP_DIR}/athrill-target-v850e2m:/root/workspace/athrill-target-v850e2m \
	-v ${ATHRILL_TOP_DIR}/athrill-device:/root/workspace/athrill-device \
	-v `pwd`/workspace:/root/workspace/ros2	\
	-it --rm --net host --name ros2-foxy ${DOCKER_IMAGE}
