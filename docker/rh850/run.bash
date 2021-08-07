#!/bin/bash

ATHRILL_TOP_DIR=$(cd ../../.. && pwd)
DOCKER_IMAGE=athrill-device-rh850-builder:v1.0.0

sudo docker run \
		-v ${ATHRILL_TOP_DIR}/athrill-device:/root/workspace/athrill-device \
		-it --rm --net host --name athrill-device-rh850 ${DOCKER_IMAGE} 
