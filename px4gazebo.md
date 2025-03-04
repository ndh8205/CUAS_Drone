# WSL2 Ubuntu 22.04 ROS2 Humble/PX4/Gazebo 시뮬레이션 환경 구축 가이드 - Part 1

## 목차

### Part 1: 기본 환경 설정 및 ROS2 설치
1. WSL2 환경 설정
2. 기본 패키지 설치
3. ROS2 Humble 설치
4. ROS2 개발 도구 설치
5. Gazebo Classic 설치

## 1. WSL2 환경 설정

### WSL2 설치 및 관리

```bash
# 현재 설치된 WSL 배포판 목록 확인
wsl --list -v

# 기존 Ubuntu 20.04 종료 및 제거 (필요한 경우)
wsl --terminate Ubuntu-22.04    # 실행 중인 WSL 인스턴스 종료
wsl --unregister Ubuntu-22.04   # WSL 배포판 완전 제거 (모든 데이터 삭제됨)

# 새로운 Ubuntu 22.04 설치
wsl --install -d Ubuntu-22.04

# 설치 후 Ubuntu 버전 확인
lsb_release -a
```

### WSL2 메모리 관리 (선택사항)
Windows에서 WSL2의 메모리 사용량을 제한하려면, 사용자 홈 디렉토리에 `.wslconfig` 파일을 생성하세요:

```plaintext
# C:\Users\<YourUsername>\.wslconfig
[wsl2]
memory=6GB
processors=4
swap=2GB
```

## 2. 기본 패키지 설치

```bash
# 시스템 업데이트
sudo apt update && sudo apt upgrade -y

# 필수 패키지 설치
sudo apt install -y \
    git \
    build-essential \
    cmake \
    libssl-dev \
    wget \
    python3-pip \
    python3-dev \
    python3-setuptools \
    python3-wheel \
    ninja-build \
    exiftool \
    astyle \
    ccache \
    clang \
    clang-tidy \
    g++ \
    gcc \
    gdb \
    make \
    rsync \
    shellcheck \
    unzip \
    xsltproc \
    zip

# Java 설치
sudo apt install -y default-jdk
echo "export JAVA_HOME=/usr/lib/jvm/java-11-openjdk-amd64" >> ~/.bashrc
source ~/.bashrc

# libasio 설치
sudo apt install -y libasio-dev
```

## 3. ROS2 Humble 설치

### Locale 설정
```bash
# locale 설정
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### ROS2 저장소 추가
```bash
# ROS2 저장소 추가
sudo apt install -y curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### ROS2 Humble 설치
```bash
# ROS2 Humble 설치
sudo apt update
sudo apt install -y ros-humble-desktop
```

## 4. ROS2 개발 도구 설치

### 기본 개발 도구 설치
```bash
# ROS2 개발 도구 및 의존성 패키지 설치
sudo apt update
sudo apt install -y \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-flake8 \
    python3-pytest-cov \
    python3-setuptools \
    python3-vcstool \
    python3-rosinstall-generator \
    ros-humble-ament-* \
    ros-humble-ros-testing \
    ros-humble-eigen3-cmake-module
```

### ROS2 환경 설정
```bash
# ROS2 환경 설정
source /opt/ros/humble/setup.bash

# rosdep 초기화 및 업데이트
sudo rosdep init || true
rosdep update

# colcon mixin 추가
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update
```

## Part 1 설치 확인

각 구성 요소가 제대로 설치되었는지 확인하세요:

```bash
# ROS2 설치 확인
ros2 --help  # ROS2 명령어 목록 확인
ros2 -h      # 위와 동일
ros2 topic -h  # 특정 명령어의 도움말 보기

# ROS2 distro 확인
# 'humble'이 출력되어야 함
printenv ROS_DISTRO    

rosdep --version
```

# WSL2 Ubuntu 22.04 ROS2 Humble/PX4/Gazebo 시뮬레이션 환경 구축 가이드 - Part 2

## 목차

### Part 2: PX4 및 DDS 설치
5. PX4 설치
6. DDS 설치
7. ROS2 작업공간 설정
8. 최종 환경 변수 설정
9. 시뮬레이션 실행 및 테스트
10. 문제 해결 가이드

## 5. PX4 설치

### PX4 Firmware 다운로드
```bash
# PX4 Firmware 클론
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive

# PX4 툴체인 설치
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh

# 환경 다시 불러오기
source ~/.bashrc
```

## 6. DDS 설치

### 기존 설치 제거
```bash
# 기존 설치 제거 (있다면)
cd ~
sudo rm -rf Fast-CDR fastcdr Fast-DDS Fast-DDS-Gen
sudo rm -rf /usr/local/include/fastcdr /usr/local/include/fastrtps
sudo rm -rf /usr/local/lib/cmake/fastcdr /usr/local/lib/cmake/fastrtps
sudo ldconfig
```

### FastCDR 설치
```bash
# FastCDR v1.0.27 설치 (Fast-DDS 2.10.1과 호환)
git clone https://github.com/eProsima/Fast-CDR.git fastcdr
cd fastcdr
git checkout v1.0.27
mkdir build && cd build
cmake ..
make
sudo cmake --build . --target install
sudo ldconfig
```

### Fast-DDS 설치
```bash
# Fast-DDS 2.10.1 설치
cd ~
git clone --recursive https://github.com/eProsima/Fast-DDS.git
cd Fast-DDS
git checkout v2.10.1
git submodule update --init --recursive

mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DTHIRDPARTY=ON -DCMAKE_INSTALL_PREFIX=/usr/local
make -j$(nproc)
sudo make install
sudo ldconfig
```

### Fast-DDS-Gen 설치
```bash
# Fast-DDS-Gen 설치
cd ~
git clone --recursive https://github.com/eProsima/Fast-DDS-Gen.git -b v2.4.0
cd Fast-DDS-Gen
./gradlew assemble
sudo ./gradlew install
```

### Micro-XRCE-DDS-Agent 설치
```bash
# Micro-XRCE-DDS-Agent 설치
cd ~
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

# 실행 파일을 시스템 경로에 복사
sudo cp MicroXRCEAgent /usr/local/bin/
sudo ldconfig

# 환경 변수 설정
echo 'export PATH=$PATH:/usr/local/bin' >> ~/.bashrc
source ~/.bashrc
```

## 7. ROS2 작업공간 설정

```bash
# ROS2 workspace 생성
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# PX4 ROS2 패키지 설치
cd src
git clone https://github.com/PX4/px4_ros_com.git
git clone https://github.com/PX4/px4_msgs.git

# workspace 루트 디렉토리로 이동
cd ~/ros2_ws

# ROS2 환경 설정
source /opt/ros/humble/setup.bash

# workspace 의존성 설치
rosdep install --from-paths src --ignore-src -r -y

# workspace 빌드
colcon build --packages-select px4_msgs px4_ros_com

# 새로 빌드한 패키지 환경 설정 적용
source install/setup.bash
```

## 8. 최종 환경 변수 설정

```bash
# 환경 변수 설정 파일 백업
cp ~/.bashrc ~/.bashrc.backup

# 환경 변수 설정 추가
cat << 'EOL' >> ~/.bashrc

# ROS2 및 Colcon 설정
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Gazebo Harmonic 설정
export GZ_VERSION=harmonic
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/PX4-Autopilot/Tools/simulation/gz/models
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/PX4-Autopilot/Tools/simulation/gz/worlds

# PX4 설정
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot/Tools/simulation/gazebo

# DDS 설정
export PATH=$PATH:/usr/local/bin
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
EOL

# 새 환경 변수 적용
source ~/.bashrc
```

## 9. 시뮬레이션 실행 및 테스트

각 단계는 **새로운 터미널**에서 실행하며, 각 터미널에서 환경 설정을 다시 로드해야 합니다.

### 터미널 1: PX4 SITL 실행
```bash
cd ~/PX4-Autopilot
source ~/.bashrc  # 환경 변수 다시 로드
make px4_sitl gz_x500
```

### 터미널 2: DDS Agent 실행
```bash
# 환경 설정 로드
source ~/.bashrc
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

MicroXRCEAgent udp4 -p 8888
```

### 터미널 3: ROS2 노드 실행
```bash
# 환경 설정 로드
source ~/.bashrc
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

cd ~/ros2_ws
ros2 run px4_ros_com sensor_combined_listener
```

## 10. 문제 해결 가이드

### "You need to have gazebo simulator installed!" 오류 해결

이 오류가 발생하는 경우, 다음 단계들을 순서대로 실행하세요:

```bash
# 1. Gazebo Classic 재설치
sudo apt-get update
sudo apt-get install --reinstall gazebo libgazebo-dev

# 2. 환경 변수 확인
echo $GAZEBO_PLUGIN_PATH
echo $GAZEBO_MODEL_PATH

# 3. 새 터미널에서 환경 설정 다시 로드
source ~/.bashrc

# 4. Gazebo와 PX4 설정 수동 로드
source /usr/share/gazebo/setup.sh
source ~/PX4-Autopilot/Tools/simulation/gazebo/setup_gazebo.bash ~/PX4-Autopilot ~/PX4-Autopilot/build/px4_sitl_default

# 5. PX4 빌드 디렉토리 정리 후 재시도
cd ~/PX4-Autopilot
make clean
make px4_sitl gazebo
```

### 유용한 디버깅 명령어

```bash

# ROS2 환경 확인
printenv | grep ROS

```

### 일반적인 문제 해결 팁

1. 항상 새로운 터미널을 열 때마다 환경 설정을 다시 로드해야 합니다.
2. 환경 변수 설정의 순서가 중요합니다. Gazebo 설정을 먼저 로드한 후 PX4 설정을 로드해야 합니다.
3. 문제가 지속되면 다음 명령으로 전체 빌드를 클린하고 다시 시도해보세요:
```bash
cd ~/PX4-Autopilot
make clean
make distclean
make px4_sitl gazebo
```
4. 각 구성 요소의 버전 호환성을 확인하세요. 특히 Fast-DDS와 관련 구성 요소들의 버전이 맞아야 합니다.



