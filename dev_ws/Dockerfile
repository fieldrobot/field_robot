FROM ubuntu:focal

#CUSTOM
ARG DEBIAN_FRONTEND=noninteractive

#locale - timezone setup
RUN apt update && \
  apt install -y locales && \
  locale-gen en_US en_US.UTF-8 && \
  update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LC_ALL=en_US.UTF-8
ENV LANG=en_US.UTF-8

#setup sources
RUN apt update && \
  apt install -y curl gnupg2 lsb-release && \
  curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg && \
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

#install ROS
ENV ROS_DISTRO foxy
RUN apt update && apt install -y ros-foxy-ros-base && \
  echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc && \
  echo "source /field_robot/dev_ws/install/setup.bash" >> ~/.bashrc

#additional recommended/necessary packages
RUN apt update && apt install -y \
  build-essential \ 
  ros-foxy-rviz2 \
  python3-colcon-common-extensions \
  python3-colcon-mixin \
  python3-rosdep \
  python3-vcstool \
  python3-argcomplete

# Finalise ROS setup
RUN rosdep init && rosdep update --rosdistro $ROS_DISTRO

# Project Dependencys
RUN apt update && apt install -y \
  ros-foxy-navigation2 \
  ros-foxy-nav2-bringup \
  ros-foxy-slam-toolbox \
  ros-foxy-behaviortree-cpp-v3 \
  ros-foxy-cv-bridge \
  ros-foxy-robot-localization \
  python3-opencv\
  dos2unix

# Install Gazebo
RUN apt update && apt install -y ros-foxy-gazebo-ros-pkgs

# Install Pip (And possible Pip dependencys)
RUN apt update && apt install -y python3-pip && \
  pip install --upgrade pip && \
  pip install tensorflow-aarch64 -f https://tf.kmtea.eu/whl/stable.html

RUN apt update && apt upgrade -y

#volumes for field_robot folder
VOLUME [ "/field_robot" ]

CMD bash