![image](https://github.com/user-attachments/assets/c4f02da6-0615-4603-97ef-119bb74ef911)# WSL2 Ubuntu 20.04 PX4/Gazebo 시뮬레이션 환경 구축 완전 가이드

## 목차

1. [WSL2 환경 설정](#1-wsl2-환경-설정)
2. [기본 패키지 설치](#2-기본-패키지-설치)
3. [Gazebo 설치](#3-gazebo-설치)
4. [ROS Noetic 설치](#4-ros-noetic-설치)
5. [환경 변수 초기 설정](#5-환경-변수-초기-설정)
6. [Catkin 작업공간 설정](#6-catkin-작업공간-설정)
7. [PX4 설치](#7-px4-설치)
8. [MAVROS 설치](#8-mavros-설치)
9. [최종 환경 변수 설정](#9-최종-환경-변수-설정)
10. [WSL2 최적화](#10-wsl2-최적화)
11. [시뮬레이션 실행](#11-시뮬레이션-실행)

## 1. WSL2 환경 설정

### 시스템 요구사항 확인
```bash
# Ubuntu 버전 확인
lsb_release -a

# WSL2 IP 주소 확인 
ip addr | grep eth0
```
### 시스템 초기화 (필요시)
```bash
wsl --terminate Ubuntu-20.04
wsl --unregister Ubuntu-20.04
wsl --install -d Ubuntu-20.04
```

## 2. 기본 패키지 설치

```bash
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
```

## 3. Gazebo 설치

```bash
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
```

## 4. ROS Noetic 설치

```bash
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
source ~/.bashrc

# Python 3 설정
echo "alias python=python3" >> ~/.bashrc
source ~/.bashrc

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
```

### 설치 확인
```bash
# ROS 버전 확인
rosversion -d

# ROS 환경 확인
printenv | grep ROS

# roscore 실행 테스트 (새 터미널에서)
roscore
```

### 주의사항
- `roscore` 실행 시 에러가 발생하면 다음을 확인하세요:
  1. ROS 환경 변수가 제대로 설정되었는지 (`echo $ROS_DISTRO`)
  2. Python 3가 기본 Python으로 설정되었는지 (`python --version`)
  3. 모든 의존성이 제대로 설치되었는지 (`rosdep check --from-paths /opt/ros/noetic`)

### 문제 해결
```bash
# 의존성 문제 발생 시
sudo apt --fix-broken install

# ROS 패키지 재설치
sudo apt install --reinstall ros-noetic-desktop-full

# rosdep 초기화 문제 시
sudo rm /etc/ros/rosdep/sources.list.d/20-default.list
sudo rosdep init
rosdep update
```

## 5. 환경 변수 초기 설정

```bash
# ROS 환경 설정
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Gazebo 환경 설정
echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc

# 변경사항 적용
source ~/.bashrc
```

## 6. Catkin 작업공간 설정

```bash
# Catkin tools 설치
sudo apt-get install python3-catkin-tools python3-osrf-pycommon

# Catkin 작업공간 생성
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init

# 작업공간 빌드
catkin build
```

## 7. PX4 설치

```bash
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
```

## 8. MAVROS 설치

MAVROS는 바이너리 설치와 소스 설치 두 가지 방법이 있습니다.

### 옵션 1: 바이너리 설치 (일반적인 사용)
```bash
# MAVROS 및 의존성 설치
sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras

# GeographicLib 데이터셋 설치
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh
```

### 옵션 2: 소스 설치 (개발 및 커스터마이징용)
```bash
# 기존 MAVROS 제거 (이전에 설치한 경우)
cd ~/catkin_ws/src
rm -rf mavros

# MAVROS 소스 설치
rosinstall_generator --rosdistro noetic mavros mavros_extras | tee /tmp/mavros.rosinstall
wstool init src /tmp/mavros.rosinstall    # 에러 발생시: wstool init -j8 . /tmp/mavros.rosinstall
wstool update -t src -j4

# 의존성 설치
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y

# GeographicLib 데이터셋 설치
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh

# 작업공간 재빌드
catkin build -j1
```

### 설치 확인
```bash
# MAVROS 버전 확인
rosversion mavros

# MAVROS 노드 실행 테스트
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```

### 바이너리 설치와 소스 설치의 차이점

**바이너리 설치**
- 장점:
  - 빠르고 간단한 설치
  - 안정적인 검증된 버전
  - 자동 의존성 처리
  - 쉬운 시스템 업데이트
- 단점:
  - 최신 기능 사용 불가
  - 코드 수정 불가
  - 제한된 버전만 사용 가능

**소스 설치**
- 장점:
  - 최신 개발 버전 사용 가능
  - 코드 수정 및 커스터마이징 가능
  - 디버깅 용이
  - 특정 버전/브랜치 선택 가능
- 단점:
  - 복잡한 설치 과정
  - 의존성 문제 가능성
  - 빌드 실패 가능성
  - 더 많은 시스템 리소스 사용

### 선택 기준
- 바이너리 설치: 일반적인 사용, 안정성 중시, 빠른 설치 필요
- 소스 설치: 코드 수정/개발 필요, 최신 기능 필요, 연구/실험용

  
## 9. 최종 환경 변수 설정

```bash
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
```

## 10. 시뮬레이션 실행

### 방법 1: PX4 SITL 단독 실행

```bash
# PX4 디렉토리로 이동
cd ~/PX4-Autopilot

# SITL 시뮬레이션 실행
make px4_sitl gazebo
```

### 방법 2: MAVROS로 실행

```bash
# MAVROS 실행
roslaunch px4 mavros_posix_sitl.launch

# 또는
roslaunch ~/PX4-Autopilot/launch/mavros_posix_sitl.launch

# 또는 다음과 같이 특정 설정으로 실행:
# 특정 기체 모델로 실행
roslaunch ~/PX4-Autopilot/launch/mavros_posix_sitl.launch vehicle:=iris_fpv_cam

# 특정 월드 파일로 실행
roslaunch ~/PX4-Autopilot/launch/mavros_posix_sitl.launch world:=$worlds/test.world

# 기체와 월드 모두 지정
roslaunch ~/PX4-Autopilot/launch/mavros_posix_sitl.launch vehicle:=iris_fpv_cam world:=$worlds/test.world
```

### 문제 해결

1. MAVROS 실행 시 GeographicLib 오류 발생하는 경우:
```bash
# GeographicLib 데이터셋 재설치
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh
```

2. setup_gazebo.bash 오류 발생 시:
```bash
# PX4를 다시 빌드
cd ~/PX4-Autopilot
DONT_RUN=1 make px4_sitl_default gazebo

# 환경 변수 재설정
source Tools/simulation/gazebo-classic/setup_gazebo.bash ~/PX4-Autopilot ~/PX4-Autopilot/build/px4_sitl_default
```

3. Launch 파일을 찾을 수 없는 경우:
```bash
# 전체 경로를 사용하여 실행
roslaunch ~/PX4-Autopilot/launch/mavros_posix_sitl.launch
```

## 주의사항

- MAVROS를 실행하기 전에 반드시 PX4 SITL이 먼저 실행되어 있어야 합니다
- 새 터미널을 열 때마다 `source ~/.bashrc` 실행이 필요합니다
- WSL2에서 GUI 애플리케이션 실행을 위해 X 서버 설정이 필요합니다
- 빌드 실패 시 먼저 메모리 관련 설정을 확인하세요
- 환경 변수 설정에서 setup_gazebo.bash는 반드시 src_dir과 build_dir 두 인자가 필요합니다


## 부록1 : ArucoMarker 추가하기

### 1. 원하는 아루코 마커를 px170 x px170으로 맞춰서 생성
### 2. 아래 코드를 클론하고 gazebo_models/ar_tags/images에 Marker18과 같이 넣으면 됨

```bash
git clone https://github.com/mikaelarguedas/gazebo_models.git

cd gazebo_models/ar_tags/scripts

nano generate_markers_model.py
# 맨 위에 빨간 글씨로 아래 코드가 적혀있을 것 (# 붙어있는게 맞음, # 지우면 안됨)
#! /usr/bin/env python
# 맨 위에 빨간 글씨를 아래 코드로 수정 (# 붙어있는게 맞음, # 지우면 안됨)
#! /usr/bin/env python3

mkdir -p ~/.gazebo/models

sudo apt-get update
sudo apt-get install imagemagick

cd ~/gazebo_models/ar_tags/scripts
chmod +x generate_markers_model.py

./generate_markers_model.py -i $HOME/gazebo_models/ar_tags/images -s 1000 -w 200

# 기존 모델의 삭제 코드는 아래와 같음
rm -rf $HOME/.gazebo/models/marker*
```
## 부록2 : ArucoMarker px170 x px170으로 맞춰서 생성하기

### 1. 원하는 아루코 마커를 https://chev.me/arucogen/ 이 사이트에 접속해서 생성
이미지의 generator에서 size를 170px로 맞추려면:
- Marker size를 29mm로 설정 (170/500 x 100 = 29)
이렇게 하면 170x170px 크기의 마커가 생성

## 부록3 : Offboard Control python code 추가하기

### 1. ROS와 PX4의 코드 규칙에 맞게 코드 작성


작성하신 드론 컨트롤 코드를 실행하기 위한 설정과 실행 방법을 안내해드리겠습니다.

1. **패키지 설정**
```bash
# Catkin 작업공간의 src 폴더에 패키지 생성
cd ~/catkin_ws/src
catkin_create_pkg drone_aruco_control rospy mavros geometry_msgs sensor_msgs cv_bridge

# 패키지 내부에 필요한 디렉토리 생성
cd drone_aruco_control
mkdir scripts
```

2. **의존성 패키지 설치**
```bash
# OpenCV와 관련 패키지 설치
sudo apt-get update
sudo apt-get install -y \
    python3-opencv \
    ros-noetic-cv-bridge \
    ros-noetic-vision-opencv \
    python3-tf \
    ros-noetic-tf
```

3. **코드 설치**
```bash
# scripts 폴더에 Python 파일 복사
cd ~/catkin_ws/src/drone_aruco_control/scripts
# drone_control.py 파일을 이 위치에 복사

# 실행 권한 부여
chmod +x drone_control.py
```
### 만약 3번을 진행하며 코드가 이미 있다면 저기에 넣고 실행권한 부여 명령어의 이름만 수정

4. **패키지 빌드**
```bash
cd ~/catkin_ws
catkin build
source devel/setup.bash


rosrun my_drone_control drone_control.py

```
