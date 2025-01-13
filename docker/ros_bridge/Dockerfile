# Use the official ROS Noetic base image
FROM osrf/ros:foxy-ros1-bridge

SHELL ["/bin/bash", "-c"]

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8 LC_ALL=C.UTF-8

# Add APT key to prevent errors
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654

# Install dependencies
RUN apt-get update && apt-get install -y \
    git \
    nano \
    libgl1-mesa-glx \
    libglu1-mesa \
    mesa-utils \
    libx11-dev \
    libxrender-dev \
    libxtst-dev

# Initialize rosdep
# RUN rosdep init && rosdep update

CMD ["/bin/bash"]

# Switch Users
RUN useradd -ms /bin/bash dev \
    && echo "dev:nickreed" | chpasswd \
    && usermod -aG sudo dev

USER dev

WORKDIR /home/dev