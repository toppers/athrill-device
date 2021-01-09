#!/bin/bash

DOCKER_IMAGE=kanetugu2015/ros2-foxy:v1.0.0

sudo docker run \
	-v `pwd`/workspace:/root/workspace	\
	-it --rm --net host --name ros2-foxy ${DOCKER_IMAGE}
