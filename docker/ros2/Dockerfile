# Use the official ROS Humble base image
FROM osrf/ros:humble-desktop-full-jammy

SHELL ["/bin/bash", "-c"]

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8 LC_ALL=C.UTF-8

# Install dependencies
RUN apt-get update && apt-get install -y \
    git \
    nano \
    python3-pip \
    python3-wstool \
    build-essential \
    libceres-dev \
    libgl1-mesa-glx \
    libglu1-mesa \
    mesa-utils \
    libx11-dev \
    libxrender-dev \
    libxtst-dev \
    ros-humble-image-proc

RUN apt install software-properties-common -y

# Initialize rosdep
# RUN rosdep init && rosdep update

CMD ["/bin/bash"]

# Switch Users
RUN useradd -ms /bin/bash dev \
    && echo "dev:nickreed" | chpasswd \
    && usermod -aG sudo dev

USER dev

WORKDIR /home/dev

