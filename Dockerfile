FROM ubuntu:latest
LABEL maintainer="Kshitij Sharma<s2155899@ed.ac.uk>"
LABEL version="0.1"
LABEL description="HYPED Dockerfile for using Ros2 Humble"

ARG ROS_DISTRO=humble
ARG INSTALL_PACKAGE=desktop

ARG DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]

RUN apt-get update -q && \
    apt-get upgrade -yq && \
    DEBIAN_FRONTEND=noninteractive apt-get install -yq --no-install-recommends wget curl git build-essential sudo lsb-release locales bash-completion tzdata && \
    rm -rf /var/lib/apt/lists/*

RUN rm /etc/apt/apt.conf.d/docker-clean

RUN apt-get update -q && \
    apt-get install -y ca-certificates curl gnupg2 lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update -q && \
    apt-get install -y ros-${ROS_DISTRO}-${INSTALL_PACKAGE} \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-rosdep python3-vcstool && \
    rosdep init && \
    rm -rf /var/lib/apt/lists/*

COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]


