FROM osrf/ros:noetic-desktop-full


# Dependencies we use, catkin tools is very good build system
# https://github.com/ethz-asl/kalibr/wiki/installation
RUN apt-get update && DEBIAN_FRONTEND=noninteractive \
	apt-get install -y \
	git wget autoconf automake nano \
	python3-dev python3-pip python3-scipy python3-matplotlib \
	ipython3 python3-wxgtk4.0 python3-tk python3-igraph python3-pyx \
	libeigen3-dev libboost-all-dev libsuitesparse-dev \
	doxygen \
	libopencv-dev \
	libpoco-dev libtbb-dev libblas-dev liblapack-dev libv4l-dev \
	python3-catkin-tools python3-osrf-pycommon

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


# Create the workspace and build kalibr in it

CMD ["/bin/bash"]

# Switch Users
RUN useradd -ms /bin/bash dev \
    && echo "dev:nickreed" | chpasswd \
    && usermod -aG sudo dev \
    && echo "dev ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER dev

WORKDIR /home/dev

ENV WORKSPACE /home/dev/kalibr_ws

RUN mkdir -p $WORKSPACE/src && \
	cd $WORKSPACE && \
	catkin init && \
	catkin config --extend /opt/ros/noetic && \
	catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN cd $WORKSPACE/src &&\
	git clone https://github.com/ori-drs/kalibr.git

RUN	cd $WORKSPACE &&\
	catkin build -j$(nproc)

RUN echo "source /opt/ros/noetic/setup.bash" >> /home/dev/.bashrc
RUN echo "export ROS_HOSTNAME=localhost" >> /home/dev/.bashrc 
RUN echo "export ROS_MASTER_URI=http://localhost:11311" >> /home/dev/.bashrc
 
# When a user runs a command we will run this code before theirs
# This will allow for using the manual focal length if it fails to init
# https://github.com/ethz-asl/kalibr/pull/346
ENTRYPOINT export KALIBR_MANUAL_FOCAL_LENGTH_INIT=1 && \
	# /bin/bash -c "source \"$WORKSPACE/devel/setup.bash\"" && \ 
	cd $WORKSPACE && \
	/bin/bash