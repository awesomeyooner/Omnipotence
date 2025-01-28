# Use the official ROS Noetic base image
FROM osrf/ros:noetic-desktop-full

SHELL ["/bin/bash", "-c"]

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8 LC_ALL=C.UTF-8

# Install dependencies
RUN apt-get update && apt-get install -y \
    git \
    nano \
    python3-pip \
    python3-catkin-tools \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
    libceres-dev \
    libgl1-mesa-glx \
    libglu1-mesa \
    mesa-utils \
    libx11-dev \
    libxrender-dev \
    libxtst-dev \
    python3-tk

# TagSLAM Deps
RUN apt install software-properties-common -y

RUN apt install python3-vcstool -y

RUN apt install python3-catkin-tools python3-osrf-pycommon -y

RUN apt-add-repository ppa:borglab/gtsam-release-4.1 -y && \
    apt update -y && \
    apt install libgtsam-dev libgtsam-unstable-dev -y

# Initialize rosdep
# RUN rosdep init && rosdep update

CMD ["/bin/bash"]

# Switch Users
RUN useradd -ms /bin/bash dev \
    && echo "dev:nickreed" | chpasswd \
    && usermod -aG sudo dev \
    && echo "dev ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER dev

WORKDIR /home/dev

RUN mkdir -p /home/dev/allan_ws/src

WORKDIR /home/dev/allan_ws/src

RUN git clone https://github.com/ori-drs/allan_variance_ros.git

WORKDIR /home/dev/allan_ws

RUN source /opt/ros/noetic/setup.bash && catkin build

# Finally, change back to home
# WORKDIR /home/dev

RUN echo "source /opt/ros/noetic/setup.bash" >> /home/dev/.bashrc
RUN echo "export ROS_HOSTNAME=localhost" >> /home/dev/.bashrc 
RUN echo "export ROS_MASTER_URI=http://localhost:11311" >> /home/dev/.bashrc

