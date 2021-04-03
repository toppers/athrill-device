#!/bin/bash

ATHRILL_TOP_DIR=$(cd ../../.. && pwd)
DOCKER_IMAGE=kanetugu2015/athrill-device-v850:v1.0.0

sudo docker run -v ${ATHRILL_TOP_DIR}/athrill:/root/workspace/athrill \
		-v ${ATHRILL_TOP_DIR}/athrill-target-v850e2m:/root/workspace/athrill-target-v850e2m \
		-v ${ATHRILL_TOP_DIR}/athrill-device:/root/workspace/athrill-device \
		-v ${ATHRILL_TOP_DIR}/hakoniwa-core:/root/workspace/hakoniwa-core \
		-it --rm --net host --name athrill-device-v850 ${DOCKER_IMAGE} 
