# ROS distribution to use
ARG ROS_DISTRO=noetic

###############################################################################################
# Base Image for #
###############################################################################################

FROM osrf/ros:${ROS_DISTRO}-desktop-full AS base
ENV ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-c"]

# Install basic apt packages
RUN apt-get update && apt-get install -y \
  software-properties-common \
  git \
  build-essential \
  cmake \
  python3-catkin-tools \
  libopencv-dev \
  g++ \
  libcanberra-gtk* \
  python3-catkin-tools \
  python3-pip \
  #python3-tk \
  wget \
  vim \
  iputils-ping \
  python3-vcstool \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
  libeigen3-dev \
  ros-${ROS_DISTRO}-hector-trajectory-server \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

# Create Catkin workspace 
RUN mkdir -p /underlay_ws/src 
WORKDIR /underlay_ws/src
COPY dependencies.repos .
RUN vcs import --recursive < dependencies.repos

WORKDIR /underlay_ws/src/Pangolin
RUN ./scripts/install_prerequisites.sh recommended || true
RUN cmake -B build 
RUN cmake --build build
RUN cmake --install build

RUN source /opt/ros/noetic/setup.bash \
  && catkin init 

# Build the base Catkin workspace
RUN cd /underlay_ws \
  && source /opt/ros/noetic/setup.bash \
  && rosdep install -y --from-paths src --ignore-src --skip-keys="catch2" \
  && catkin build --verbose

# Remove display warnings
RUN mkdir /tmp/runtime-root
ENV XDG_RUNTIME_DIR "/tmp/runtime-root"
ENV NO_AT_BRIDGE 1

# Set up the work directory and entrypoint
WORKDIR /underlay_ws
COPY ./docker/entrypoint.sh /

# Alias python3 as python
RUN ln -s /usr/bin/python3 /usr/bin/python

ENTRYPOINT [ "/entrypoint.sh" ]


###############################################################################################
# Overlay Image #
###############################################################################################
FROM base AS overlay
# Create an overlay Catkin workspace
RUN source /opt/ros/noetic/setup.bash \
  && mkdir -p /overlay_ws/src \
  && cd /overlay_ws \ 
  && catkin init
COPY ../orb_slam3_ros/ /overlay_ws/src/orb_slam3_ros/
#COPY ./some_other_folder/ /overlay_ws/src/some_other_folder/
WORKDIR /overlay_ws
#RUN cd ./src && git clone https://github.com/thien94/orb_slam3_ros.git
RUN source /opt/ros/noetic/setup.bash \
  source /underlay_ws/devel/setup.bash \
  && catkin build

# Set up the work directory and entrypoint
COPY ./docker/entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]


################################################################################################
## Development Image #
################################################################################################
FROM overlay AS dev

# Dev container arguments
ARG USERNAME=devuser
ARG UID=1000
ARG GID=${UID}

# Install extra tools for development
RUN apt-get update && apt-get install -y --no-install-recommends \
  gdb gdbserver nano

# Create new user and home directory
RUN groupadd --gid $GID $USERNAME \
  && useradd --uid ${GID} --gid ${UID} --create-home ${USERNAME} \
  && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
  && chmod 0440 /etc/sudoers.d/${USERNAME} \
  && mkdir -p /home/${USERNAME} \
  && chown -R ${UID}:${GID} /home/${USERNAME}

# Set the ownership of the overlay workspace to the new user
RUN chown -R ${UID}:${GID} /overlay_ws/


# Set the user and source entrypoint in the user's .bashrc file
USER ${USERNAME}
RUN echo "source /entrypoint.sh" >> /home/${USERNAME}/.bashrc