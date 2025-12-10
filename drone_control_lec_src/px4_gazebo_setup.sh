#!/bin/bash

# WSL2 Ubuntu 20.04 PX4/Gazebo 시뮬레이션 환경 구축 스크립트
# 2번 ~ 9번 설치 과정

set -e  # 에러 발생 시 스크립트 중단

echo "============================================"
echo "2. 기본 패키지 설치"
echo "============================================"

# 시스템 업데이트
sudo apt update && sudo apt upgrade -y

# 필수 패키지 설치
sudo apt install -y \
    git \
    python3-pip \
    python3-dev \
    ninja-build \
    exiftool \
    astyle \
    build-essential \
    ccache \
    clang \
    clang-tidy \
    cmake \
    g++ \
    gcc \
    gdb \
    make \
    ninja-build \
    python3-dev \
    python3-pip \
    python3-setuptools \
    python3-wheel \
    rsync \
    shellcheck \
    unzip \
    xsltproc \
    zip

echo "============================================"
echo "3. Gazebo 설치"
echo "============================================"

# Gazebo 11 저장소 추가
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

# Gazebo 11 설치
sudo apt update
sudo apt install -y \
    gazebo11 \
    libgazebo11-dev \
    gazebo11-plugin-base \
    gazebo11-common

# 추가 의존성 패키지 설치
sudo apt install -y \
    python3-jinja2 \
    python3-pip \
    python3-cerberus \
    python3-numpy \
    python3-yaml \
    python3-setuptools \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-ugly

echo "============================================"
echo "4. ROS Noetic 설치"
echo "============================================"

# ROS 저장소 추가
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# 키 설정
sudo apt install -y curl gnupg lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# 패키지 목록 업데이트
sudo apt update

# ROS Noetic 전체 설치
sudo apt install -y ros-noetic-desktop-full

# ROS 개발 도구 및 의존성 설치
sudo apt install -y \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    python3-catkin-tools \
    python3-osrf-pycommon \
    python-is-python3 \
    build-essential \
    ros-noetic-rqt* \
    ros-noetic-rviz \
    ros-noetic-plotjuggler-ros

# ROS 추가 패키지 설치 (PX4/MAVROS 관련)
sudo apt install -y \
    ros-noetic-tf2-ros \
    ros-noetic-tf2-eigen \
    ros-noetic-navigation \
    ros-noetic-robot-state-publisher \
    ros-noetic-joint-state-publisher \
    ros-noetic-joint-state-publisher-gui \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control

# rosdep 초기화 (이미 초기화된 경우 에러가 발생할 수 있음)
sudo rosdep init || true
rosdep update

# ROS 환경 설정
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Python 3 설정
echo "alias python=python3" >> ~/.bashrc

# 작업 공간 생성을 위한 catkin 설치 확인
sudo apt install -y python3-catkin-tools python3-pip

# pip를 통한 추가 Python 패키지 설치
pip3 install --user \
    osrf-pycommon \
    numpy \
    empy \
    toml \
    packaging \
    nose

echo "============================================"
echo "5. 환경 변수 초기 설정"
echo "============================================"

# ROS 환경 설정
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Gazebo 환경 설정
echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc

# 현재 쉘에서 ROS 환경 로드 (catkin build를 위해 필요)
source /opt/ros/noetic/setup.bash

echo "============================================"
echo "6. Catkin 작업공간 설정"
echo "============================================"

# Catkin tools 설치
sudo apt-get install python3-catkin-tools python3-osrf-pycommon

# Catkin 작업공간 생성
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws

# ROS noetic을 extend하도록 설정 후 초기화
catkin init
catkin config --extend /opt/ros/noetic

# 작업공간 빌드
catkin build

echo "============================================"
echo "7. PX4 설치"
echo "============================================"

# PX4 Firmware 클론
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive

# PX4 툴체인 설치
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh

# 환경 다시 불러오기
source ~/.bashrc

# PX4 SITL Gazebo 패키지 설치
cd ~/catkin_ws/src
git clone --recursive https://github.com/PX4/PX4-SITL_gazebo-classic.git

echo "============================================"
echo "8. MAVROS 설치 (바이너리 설치)"
echo "============================================"

# MAVROS 및 의존성 설치
sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras

# GeographicLib 데이터셋 설치
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh

echo "============================================"
echo "9. 최종 환경 변수 설정"
echo "============================================"

# 기존 설정 백업
cp ~/.bashrc ~/.bashrc.backup

# PX4 빌드 (환경 설정 전에 필요)
cd ~/PX4-Autopilot
DONT_RUN=1 make px4_sitl_default gazebo

# 환경 변수 설정 추가
cat << 'EOL' >> ~/.bashrc

# ROS 및 Catkin 설정
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

# Gazebo 설정
source /usr/share/gazebo/setup.sh

# PX4 설정
source ~/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash ~/PX4-Autopilot ~/PX4-Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot/Tools/simulation/gazebo-classic
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins
EOL

# 새 환경 변수 적용
source ~/.bashrc

echo "============================================"
echo "설치 완료!"
echo "============================================"
echo "새 터미널을 열거나 'source ~/.bashrc'를 실행하세요."
echo ""
echo "시뮬레이션 실행 방법:"
echo "  방법 1: cd ~/PX4-Autopilot && make px4_sitl gazebo"
echo "  방법 2: roslaunch px4 mavros_posix_sitl.launch"
