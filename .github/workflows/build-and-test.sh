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
                   python3-pip \
                   python3-pytest-cov \
                   python3-rosdep \
                   python3-rosdep \
                   python3-setuptools \
                   python3-vcstool \
                   wget
# TODO: get from apt when upgrading to Jammy
python3 -m pip install -U \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest \
  setuptools

cd $COLCON_WS_SRC
cp -r $GITHUB_WORKSPACE $COLCON_WS_SRC

rosdep init
rosdep update
rosdep install --from-paths $COLCON_WS_SRC -i -y --rosdistro $ROS_DISTRO

# Build
source /opt/ros/$ROS_DISTRO/setup.bash
cd $COLCON_WS
colcon build --event-handlers console_direct+

# Test
colcon test --event-handlers console_direct+
colcon test-result
