FROM ros:noetic-ros-base-focal

ARG USERNAME=obadah
ARG USER_UID=1000
ARG USER_GID=1000

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    build-essential \
    git \
    iputils-ping \
    net-tools \
    python3-serial \
    python3-rosdep \
    sudo \
    ros-noetic-amcl \
    ros-noetic-map-server \
    ros-noetic-message-generation \
    ros-noetic-move-base \
    ros-noetic-navigation \
    ros-noetic-ridgeback-desktop \
    ros-noetic-ridgeback-simulator \
    ros-noetic-gazebo-ros \
    ros-noetic-gazebo-ros-control \
    ros-noetic-slam-gmapping \
    ros-noetic-teleop-twist-keyboard \
    ros-noetic-tf2-tools \
    ros-noetic-rqt \
    ros-noetic-rqt-common-plugins \
    ros-noetic-rviz \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /tmp/puma_ws/src \
    && git clone --depth 1 --branch melodic-devel https://github.com/clearpathrobotics/puma_motor_driver.git /tmp/puma_motor_driver \
    && ln -s /tmp/puma_motor_driver/puma_motor_msgs /tmp/puma_ws/src/puma_motor_msgs \
    && . /opt/ros/noetic/setup.sh \
    && cd /tmp/puma_ws \
    && catkin_make install -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic \
    && rm -rf /tmp/puma_ws /tmp/puma_motor_driver

RUN groupadd --gid "${USER_GID}" "${USERNAME}" \
    && useradd --uid "${USER_UID}" --gid "${USER_GID}" -m "${USERNAME}" \
    && usermod -aG dialout "${USERNAME}" \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > "/etc/sudoers.d/${USERNAME}" \
    && chmod 0440 "/etc/sudoers.d/${USERNAME}"

COPY config/ros-noetic-entrypoint.sh /ros-noetic-entrypoint.sh
RUN chmod +x /ros-noetic-entrypoint.sh

USER "${USERNAME}"
WORKDIR "/home/${USERNAME}/ws"

ENTRYPOINT ["/ros-noetic-entrypoint.sh"]
CMD ["bash"]
