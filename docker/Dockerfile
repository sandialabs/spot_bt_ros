### Create base ROS Humble image. ###
FROM ros:humble AS base

# Finish environment setup
ENV LANG=en_US.UTF-8 \
    QT_X11_NO_MITSHM=1 \
    TZ=US/Mountain

# Install bootstrap tools, dev tools, and extra dependencies.
RUN ln -snf /usr/share/zoneinfo/${TZ} /etc/localtime && echo ${TZ} > /etc/timezone \
    && apt-get update \
    && apt-get install --no-install-recommends -y \
        bash-completion \
        bc \
        curl \
        dos2unix \
        libfreetype6-dev \
        libglib2.0-0  \
        libprotobuf-dev \
        libsm6 \
        libtool \
        libxext6 \
        libyaml-cpp-dev \
        python3-apt \
        python3-argcomplete \
        python3-dev \
        python3-distutils \
        python3-pip \
        python3-protobuf \
        python3-pytest-cov \
        python3-setuptools \
        python3-wheel \
        ros-${ROS_DISTRO}-py-trees \
        ros-${ROS_DISTRO}-py-trees-ros \
        wget \
    && apt-get -y autoremove \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Copy config into /etc
COPY ["bash.bashrc", "/etc/"]

# Update pip and install Spot python SDK
ARG SDK_VERSION=4.1.1
RUN python3 -m pip install --no-cache-dir -U pip \
    && python3 -m pip install --force-reinstall \
        "setuptools==59.6.0" \
    && python3 -m pip install --no-cache-dir \
        aiortc==1.5.0 \
        bosdyn-api==${SDK_VERSION} \
        bosdyn-choreography-client==${SDK_VERSION} \
        bosdyn-client==${SDK_VERSION} \
        bosdyn-core==${SDK_VERSION} \
        bosdyn-mission==${SDK_VERSION} \
        grpcio==1.59.3 \
        image==1.5.33 \
        inflection==0.5.1 \
        numpy==1.24.1 \
        Pillow \
        protobuf==4.22.1 \
        py_trees \
        scipy \
    && python3 -m pip cache purge

# Install bosdyn_msgs (automatic conversions of BD protobufs to ROS messages)
ARG ARCH=amd64
ARG MSG_VERSION=4.1.1
RUN wget -q -O /tmp/ros-humble-bosdyn_msgs_${MSG_VERSION}-jammy_${ARCH}.run https://github.com/bdaiinstitute/bosdyn_msgs/releases/download/${MSG_VERSION}/ros-humble-bosdyn_msgs_${MSG_VERSION}-jammy_${ARCH}.run \
    && chmod +x /tmp/ros-humble-bosdyn_msgs_${MSG_VERSION}-jammy_${ARCH}.run \
    &&  yes Y | /tmp/ros-humble-bosdyn_msgs_${MSG_VERSION}-jammy_${ARCH}.run \
    && rm /tmp/ros-humble-bosdyn_msgs_${MSG_VERSION}-jammy_${ARCH}.run

# Install spot-cpp-sdk
RUN wget -q -O /tmp/spot-cpp-sdk_${SDK_VERSION}_${ARCH}.deb https://github.com/bdaiinstitute/spot-cpp-sdk/releases/download/v${SDK_VERSION}/spot-cpp-sdk_${SDK_VERSION}_${ARCH}.deb \
    && dpkg -i /tmp/spot-cpp-sdk_${SDK_VERSION}_${ARCH}.deb \
    && rm /tmp/spot-cpp-sdk_${SDK_VERSION}_${ARCH}.deb

#  Force ROS colored terminal output
ENV RCUTILS_COLORIZED_OUTPUT=1

# Create entrypoint
COPY ["entrypoint.sh", "/entrypoint.sh"]
ENTRYPOINT ["/entrypoint.sh"]

### Create an overlay image containing the spot_ros2 driver. ###
FROM base AS overlay

# Build spot_ros2
RUN mkdir -p /spot_ws/src
WORKDIR /spot_ws/src
RUN git clone --recursive https://github.com/bdaiinstitute/spot_ros2.git \
    && git clone https://github.com/sandialabs/spot_bt_ros.git
SHELL ["/bin/bash", "-c"]
WORKDIR /spot_ws
RUN source /opt/ros/humble/setup.bash \
    && colcon build --symlink-install --packages-ignore proto2ros_tests \
    && source install/local_setup.bash

# Create entrypoint
COPY ["entrypoint.sh", "/entrypoint.sh"]
ENTRYPOINT ["/entrypoint.sh"]

### Create an visualization image from the base ROS image. ###
FROM overlay AS viz

# Install extra visualization deps and tools
RUN apt-get update \
    && apt-get install --no-install-recommends -y \
        ffmpeg \
        graphviz \
        imagemagick \
        libjpeg-dev \
        libjpeg8-dev \
        python3-opencv \
        ros-${ROS_DISTRO}-joint-state-publisher-gui \
        ros-${ROS_DISTRO}-rviz2 \
    && apt-get -y autoremove \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Create entrypoint
COPY ["entrypoint.sh", "/entrypoint.sh"]
ENTRYPOINT ["/entrypoint.sh"]

### Create development image from overlay ROS image. ###
FROM overlay AS devel

ARG USERNAME=spot
ARG UID=1000
ARG GID=1000

# Copy config into /etc
COPY ["tmux.conf", "/etc/tmux.conf"]

# Install extra development tools
RUN apt-get update \
    && apt-get install --no-install-recommends -y \
        gdb \
        gdbserver \
        htop \
        nano \
        sudo \
        tmux \
        unzip \
    && apt-get -y autoremove \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Install python development tools
RUN python3 -m pip install --no-cache-dir \
        black \
        pylint \
    && python3 -m pip cache purge

# Create new user with home directory
RUN groupadd --gid $GID $USERNAME \
    && useradd --uid ${UID} --gid ${GID} --create-home ${USERNAME} \
    && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME} \
    && mkdir -p /home/${USERNAME} \
    && chown -R ${UID}:${GID} /home/${USERNAME}

# Set the ownership of the overlay workspace to the new user
RUN chown -R ${UID}:${GID} /spot_ws/

# Set the user and source entrypoint in the user's .bashrc file
USER ${USERNAME}
WORKDIR /home/${USERNAME}
RUN echo "source /entrypoint.sh" >> /home/${USERNAME}/.bashrc
