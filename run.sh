#!/bin/bash

# Configuration
IMAGE_NAME="panda-gesture:v1.0-built"
CONTAINER_NAME="moveit_humble"
DOCKERFILE_PATH="."  # Adjust this if your Dockerfile is in a different directory
PROJECT_DIR=$(pwd)
PROJECT_NAME=$(basename "$PROJECT_DIR")

# Check if the Docker image exists
if ! docker image inspect "$IMAGE_NAME" > /dev/null 2>&1; then
    echo "Docker image $IMAGE_NAME not found. Building the image..."
    docker build -t "$IMAGE_NAME" "$DOCKERFILE_PATH"
    if [ $? -ne 0 ]; then
        echo "Failed to build the Docker image."
        exit 1
    fi
fi

# Check if the container is already running
if docker ps --format '{{.Names}}' | grep -q "^$CONTAINER_NAME$"; then
    echo "Attaching to the running container: $CONTAINER_NAME"
    docker exec -it "$CONTAINER_NAME" bash
    exit 0
fi

# Allow X server access for GUI applications
xhost +local:docker > /dev/null 2>&1

# Run the Docker container
echo "Starting the Docker container..."
# Source the ROS 2 setup script
echo "ros2 sourced..."
docker run \
    --rm \
    -it \
    --gpus all \
    --env NVIDIA_DRIVER_CAPABILITIES=all \
    --env TERM="xterm-color" \
    --env DISPLAY="$DISPLAY" \
    --env QT_DEBUG_PLUGINS=1 \
    --env QT_X11_NO_MITSHM=1 \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --volume "$HOME/.Xauthority:/root/.Xauthority:rw" \
    --volume "$HOME:/home:rw" \
    --volume "$PROJECT_DIR:/$PROJECT_NAME" \
    --volume "/mnt:/mnt:rw" \
    --volume "/media:/media:rw" \
    --workdir "/$PROJECT_NAME" \
    --hostname "$HOSTNAME" \
    --name "$CONTAINER_NAME" \
    --privileged \
    --net host \
    "$IMAGE_NAME" bash
