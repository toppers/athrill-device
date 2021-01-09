DOCKER_IMAGE=kanetugu2015/ros2-foxy
DOCKER_TAG=v1.0.0
sudo docker build -t ${DOCKER_IMAGE}:${DOCKER_TAG} -f Dockerfile .
