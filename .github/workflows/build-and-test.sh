#!/bin/bash
set -ev

export COLCON_WS=~/ws
export COLCON_WS_SRC=${COLCON_WS}/src
export DEBIAN_FRONTEND=noninteractive
export ROS_PYTHON_VERSION=3

mkdir -p $COLCON_WS_SRC

apt update -qq
apt install -qq -y lsb-release wget curl build-essential

echo "deb http://packages.ros.org/ros2-testing/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-testing.list
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
apt-get update -qq
apt-get install -y build-essential \
                   cmake \
                   git \
                   python3-colcon-common-extensions \
                   python3-colcon-common-extensions \
                   python3-flake8 \
                   python3-flake8-blind-except \
                   python3-flake8-builtins \
                   python3-flake8-class-newline \
                   python3-flake8-comprehensions \
                   python3-flake8-deprecated \
                   python3-flake8-docstrings \
                   python3-flake8-import-order \
                   python3-flake8-quotes \
                   python3-pytest \
                   python3-pytest-cov \
                   python3-pytest-repeat \
                   python3-pytest-rerunfailures \
                   python3-rosdep \
                   python3-rosdep \
                   python3-setuptools \
                   python3-setuptools \
                   python3-vcstool \
                   wget

cd $COLCON_WS_SRC
cp -r $GITHUB_WORKSPACE $COLCON_WS_SRC

rosdep init
rosdep update
rosdep install --from-paths $COLCON_WS_SRC -i -y --rosdistro $ROS_DISTRO

# Build
source /opt/ros/$ROS_DISTRO/setup.bash
cd $COLCON_WS
colcon build --event-handlers console_direct+

source $COLCON_WS/install/local_setup.bash

# Test
colcon test --event-handlers console_direct+
colcon test-result
