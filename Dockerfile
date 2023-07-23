FROM ubuntu:focal

LABEL maintainer="Liliia Hoefner <hoefner.l@gmx.de>"

# Set locale
RUN apt update && \
    apt install locales -y && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8

# Setup Sources
RUN apt install software-properties-common -y && \
    add-apt-repository universe && \
    apt update && \
    apt install curl -y && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    bash -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null'

# ROS 2 packages install (ROS-Base)
RUN apt update && \
    apt upgrade -y && \
    apt install ros-foxy-ros-base python3-argcomplete -y && \
    apt install ros-dev-tools -y

# Sourcing the setup script
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc

# Copy the task_package from the host's src directory to the container
COPY . /root/ros2_ws/src

RUN apt install python3-rosdep -y && \
    rosdep init && \
    rosdep update && \
    apt install python3-colcon-common-extensions -y && \
    apt-get update && \
    . /opt/ros/foxy/setup.sh && \
    rosdep install -i --from-path /root/ros2_ws/src --rosdistro foxy -y && \
    . /opt/ros/foxy/setup.sh && \
    cd /root/ros2_ws && \
    colcon build --packages-select task_package && \
    echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc && \
    apt clean


WORKDIR /root/ros2_ws

ENTRYPOINT ["/bin/bash", "-c", "source /root/ros2_ws/install/setup.bash && ros2 run task_package talker"]

