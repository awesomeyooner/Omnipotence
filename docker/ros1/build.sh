#!/bin/bash

IMAGE_NAME="omnipotence"
TAG="noetic"

docker build -t ${IMAGE_NAME}:${TAG} .