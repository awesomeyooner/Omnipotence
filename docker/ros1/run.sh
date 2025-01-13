#!/bin/bash
IMAGE_NAME="omnipotence"
TAG="noetic"
CONTAINER_NAME="container_run"

while test $# -gt 0; do
  case "$1" in
    -h|--help)
      echo "my application!"
      echo " "
      echo "options:"
      echo "-h, --help                show brief help"
      echo "-n, --name=NAME       specify the name of the container"
      exit 0
      ;;

    -n)
      shift
      if test $# -gt 0; then # If its not empty
        CONTAINER_NAME=${1}
        echo "Container Name: ${CONTAINER_NAME}"
      else
        echo "nothing specified!"
        exit 1
      fi
      shift
      ;;
    --name*)
      CONTAINER_NAME=`echo $1 | sed -e 's/^[^=]*=//g'`
      echo "Container Name: ${CONTAINER_NAME}"
      shift
      ;;

    *)
      break
      ;;
    
  esac
done

xhost +local:docker

docker run -it --rm \
    --env DISPLAY=$DISPLAY \
    --env LIBGL_ALWAYS_INDIRECT=0 \
    --volume /tmp/.X11-unix:/tmp/.X11-unix \
    --volume /dev/dri:/dev/dri \
    --name ${IMAGE_NAME}_${TAG}_${CONTAINER_NAME} \
    --net=host \
    ${IMAGE_NAME}:${TAG}