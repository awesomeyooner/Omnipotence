#!/bin/bash

IMAGE_NAME="omnipotence"
TAG="foxy_bridge"

docker build -t ${IMAGE_NAME}:${TAG} .