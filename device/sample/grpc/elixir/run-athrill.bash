#!/bin/bash

ATHRILL_TOP_DIR=$(cd ../../../../../ && pwd)
DOCKER_IMAGE=athrill-device-v850-elixir:v1.0.0

sudo docker run -v ${ATHRILL_TOP_DIR}/athrill:/root/workspace/athrill \
		-v ${ATHRILL_TOP_DIR}/athrill-target-v850e2m:/root/workspace/athrill-target-v850e2m \
		-v ${ATHRILL_TOP_DIR}/athrill-device:/root/workspace/athrill-device \
		-v $(pwd)/athrill_device_grpc_ex:/root/workspace/athrill_device_grpc_ex \
		-it --rm --net host --name athrill-device-v850_elixir ${DOCKER_IMAGE}
