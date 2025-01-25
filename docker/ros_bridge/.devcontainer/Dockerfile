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
    && usermod -aG sudo dev \
    && echo "dev ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER dev

WORKDIR /home/dev

RUN git clone https://awesomeyooner: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@github.com/awesomeyooner/Omnipotence.git
RUN cd Omnipotence/ros_bridge && sudo chmod +x build.sh && ./build.sh

WORKDIR /home/dev/Omnipotence/ros_bridge/bridge_ws

RUN echo "source /opt/ros/foxy/setup.bash" >> /home/dev/.bashrc
RUN echo "export ROS_HOSTNAME=localhost" >> /home/dev/.bashrc 
RUN echo "export ROS_MASTER_URI=http://localhost:11311" >> /home/dev/.bashrc

ENTRYPOINT [ "source install/setup.bash" ]