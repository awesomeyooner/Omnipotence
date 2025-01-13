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
    libxtst-dev

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
    && usermod -aG sudo dev

USER dev

WORKDIR /home/dev

# Clone TagSLAM Repository
RUN git clone https://github.com/berndpfrommer/tagslam_root.git

# Set directory to the tagslam repo
WORKDIR /home/dev/tagslam_root

# Build TagSLAM
RUN vcs import --recursive < tagslam_root.repos
RUN source /opt/ros/noetic/setup.bash && \
    catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo && \
    catkin build

COPY ../tagslam /home/dev/

# Finally, change back to home
WORKDIR /home/dev

