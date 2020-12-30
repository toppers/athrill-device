#!/bin/bash

DOCKER_IMAGE=kanetugu2015/athrill-device-v850:v1.0.0

sudo docker run -it --rm --net host --name athrill-device-v850 ${DOCKER_IMAGE} 
