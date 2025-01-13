#!/bin/bash

IMAGE_NAME="omnipotence"
TAG="humble"

docker build -t ${IMAGE_NAME}:${TAG} .