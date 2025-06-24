# PX4 SITL + ROS 2 Humble + Gazebo (Classic & Harmonic) VTOL 시뮬레이션 완전 가이드

**작성일**: 2025-06-23  
**대상**: WSL2 Ubuntu 22.04에서 PX4 SITL + ROS 2 Humble + 최신 Gazebo(Versions: Classic/Harmonic) VTOL 시뮬레이션을 구축·실행하려는 사용자  
**필요 사양**: Windows 10/11, WSL2, 16 GB 이상 RAM, 50 GB 이상 여유 디스크

---

## 목차

1. [Windows 준비](#1-windows-준비)  
2. [WSL2 & Ubuntu 22.04 설치](#2-wsl2--ubuntu2204-설치)  
3. [Ubuntu 초기 설정](#3-ubuntu-초기-설정)  
4. [개발 도구 설치](#4-개발-도구-설치)  
5. [PX4 SITL 설치](#5-px4-sitl-설치)  
6. [ROS 2 Humble 설치](#6-ros-2-humble-설치)  
7. [Gazebo 설치 (Classic vs Harmonic)](#7-gazebo-설치-classic-vs-harmonic)  
8. [PX4–ROS 2 통합](#8-px4–ros-2-통합)  
9. [VTOL 시뮬레이션 실행](#9-vtol-시뮬레이션-실행)  
10. [Fast DDS(Fast RTPS) 설정](#10-fast-ddsfast-rtps-설정)  
11. [QGroundControl 설정](#11-qgroundcontrol-설정)  
12. [문제 해결](#12-문제-해결)  
13. [백업 & 관리](#13-백업--관리)  
14. [빠른 시작](#14-빠른-시작)

---

## 1. Windows 준비

### 1.1 기능 활성화 (관리자 PowerShell)
```powershell
dism /online /enable-feature /featurename:Microsoft-Windows-Subsystem-Linux /all /norestart
dism /online /enable-feature /featurename:VirtualMachinePlatform /all /norestart
# (선택) Hyper-V
dism /online /enable-feature /featurename:Microsoft-Hyper-V /all /norestart
Restart-Computer
````

### 1.2 WSL2 커널 업데이트

```powershell
wsl --update
wsl --set-default-version 2
```

---

## 2. WSL2 & Ubuntu 22.04 설치

### 2.1 설치

```powershell
wsl --install -d Ubuntu-22.04
```

### 2.2 첫 실행

* 사용자명·비밀번호 설정

---

## 3. Ubuntu 초기 설정

### 3.1 시스템 업데이트·로케일·타임존

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y locales
sudo locale-gen en_US.UTF-8 ko_KR.UTF-8
sudo update-locale LANG=en_US.UTF-8
sudo timedatectl set-timezone Asia/Seoul
```

### 3.2 WSL2 특화 (`/etc/wsl.conf`)

```bash
sudo tee /etc/wsl.conf > /dev/null <<EOF
[boot]
systemd=true

[interop]
appendWindowsPath=true
EOF
```

---

## 4. 개발 도구 설치

```bash
# 필수 빌드 툴
sudo apt install -y build-essential cmake ninja-build pkg-config \
    libssl-dev libxml2-dev libxslt1-dev libyaml-dev

# Python
sudo apt install -y python3 python3-venv python3-pip python3-dev python3-wheel
python3 -m pip install --upgrade pip

# Git·유틸
sudo apt install -y git vim htop tree unzip xz-utils
git config --global init.defaultBranch main
```

---

## 5. PX4 SITL 설치

```bash
# 소스 클론
mkdir -p ~/workspace/px4 && cd ~/workspace/px4
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot

# 서브모듈 동기화·초기화
git submodule sync --recursive
git submodule update --init --recursive

# 의존성 설치
bash Tools/setup/ubuntu.sh

# 빌드 테스트
make px4_sitl_default none
```

---

## 6. ROS 2 Humble 설치

### 6.1 로케일 설정

```bash
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### 6.2 ROS 2 저장소 키 & 리포지터리 추가

```bash
sudo apt install -y curl gnupg2 lsb-release

# 1) GPG 키 가져오기 (NO_PUBKEY F42ED6FBAB17C654 해결)
curl -sSL 'https://keyserver.ubuntu.com/pks/lookup?op=get&search=0xF42ED6FBAB17C654' \
  | gpg --dearmor \
  | sudo tee /usr/share/keyrings/ros-archive-keyring.gpg > /dev/null

# 2) apt 저장소 등록
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu \
$(source /etc/os-release && echo $UBUNTU_CODENAME) main" \
| sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
```

### 6.3 ROS 2 Humble & 필수 도구 설치

```bash
# Desktop + GUI
sudo apt install -y ros-humble-desktop

# 빌드/관리 도구
sudo apt install -y python3-colcon-common-extensions python3-vcstool \
                    python3-rosdep python3-pip

# (선택) 추가 개발 도구
sudo apt install -y python3-flake8 python3-pytest-cov \
                    ros-humble-ament-lint ros-humble-ros-testing
```

### 6.4 환경 설정 & 초기화

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo rosdep init || true
rosdep update

# (권장) colcon mixin 설정
colcon mixin add default \
  https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update
```

### 6.5 설치 확인

```bash
ros2 --help
ros2 topic list
printenv ROS_DISTRO   # -> humble
```

---

## 7. Gazebo 설치 (Classic vs Harmonic)

### 7.1 Gazebo Classic (gazebo11)

```bash
sudo apt update
sudo apt install -y gazebo libgazebo-dev
sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros
```

### 7.2 Gazebo Harmonic (Ignition 최신판)

```bash
# 1) OSRF 저장소 추가
sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
  -o /usr/share/keyrings/osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/osrf-archive-keyring.gpg] \
http://packages.osrfoundation.org/gazebo/ubuntu-stable \
$(lsb_release -cs) main" \
| sudo tee /etc/apt/sources.list.d/gazebo-stable.list

sudo apt update

# 2) Harmonic 엔진 & ROS2 브릿지 설치
sudo apt install -y gz-harmonic
sudo apt install -y ros-humble-ros-gzharmonic \
                    ros-humble-ros-gzharmonic-bridge \
                    ros-humble-ros-gzharmonic-msgs \
                    ros-humble-ros-gzharmonic-plugins

# 3) 환경 변수 등록
echo 'export GZ_VERSION=harmonic' >> ~/.bashrc
source ~/.bashrc
```

---

## 8. PX4–ROS 2 통합

```bash
# 워크스페이스 생성
mkdir -p ~/workspace/ros2/px4_ros2_ws/src && cd ~/workspace/ros2/px4_ros2_ws/src

# PX4 메시지·통신 패키지
git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/PX4/px4_ros_com.git

# micro-ROS Agent 소스 빌드
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
git checkout v2.4.2

# 의존성 설치·빌드
cd ~/workspace/ros2/px4_ros2_ws
rosdep install -i --from-path src --rosdistro humble -y
colcon build --symlink-install --packages-select microxrcedds_agent

# 환경 설정
echo "source ~/workspace/ros2/px4_ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 9. VTOL 시뮬레이션 실행

### 9.1 빌드 가능한 시뮬레이션 타겟 확인

```bash
cd ~/workspace/px4/PX4-Autopilot
make help | grep gazebo
```

### 9.2 Gazebo Classic VTOL 실행

```bash
cd ~/workspace/px4/PX4-Autopilot
make distclean
make px4_sitl_default gazebo-classic_standard_vtol
```

### 9.3 Gazebo Harmonic VTOL 실행

```bash
cd ~/workspace/px4/PX4-Autopilot
make distclean
# Harmonic 엔진용 타겟
make px4_sitl gz_standard_vtol
# 헤드리스 모드(선택)
HEADLESS=1 make px4_sitl gz_standard_vtol
```

---

## 10. Fast DDS(Fast RTPS) 설정

```bash
sudo apt update
sudo apt install -y ros-humble-rmw-fastrtps-cpp
echo 'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp' >> ~/.bashrc
source ~/.bashrc
```

---

## 11. QGroundControl 설정

### 11.1 WSL2 IP 확인

```bash
ip addr | grep eth0 | grep inet
# 또는
hostname -I
```

예: `172.18.46.131`

### 11.2 Comm Links 구성 (QGC Windows)

1. QGC → **Settings** → **Comm Links**
2. `+` 클릭

   * **Type**: UDP
   * **Listening Port**: 14550
   * **Target Host**: *WSL2 IP* (예: `172.18.46.131`)
   * **Target Port**: 14550
3. **Save** → 더블클릭하여 **Connected** 확인

---

## 12. 문제 해결

* **PX4 빌드 실패**: `git submodule update --init --recursive` → `make distclean` → 재빌드
* **Cyclone DDS 에러**: Fast DDS 전환 또는 `~/.config/cyclonedds/cyclonedds.xml` 수정
* **포트 막힘**: Windows 방화벽에서 UDP 14550,14540 등 허용
* **WSL2 리소스 부족**: `wsl --shutdown` → 재시작

---

## 13. 백업 & 관리

```bash
# WSL2 전체 백업
wsl --export Ubuntu-22.04 ~/Ubuntu-22.04-$(date +%Y%m%d).tar

# 복원
wsl --unregister Ubuntu-22.04
wsl --import Ubuntu-22.04 <설치경로> ~/Ubuntu-22.04-backup.tar
```

---

## 14. 빠른 시작

* **터미널 1**: MicroXRCEAgent

  ```bash
  source ~/workspace/ros2/px4_ros2_ws/install/setup.bash
  ~/workspace/ros2/px4_ros2_ws/install/microxrcedds_agent/bin/MicroXRCEAgent udp4 -p 8888
  ```
* **터미널 2**: PX4 SITL VTOL

  ```bash
  cd ~/workspace/px4/PX4-Autopilot
  # Classic
  make px4_sitl_default gazebo-classic_standard_vtol
  # Harmonic
  make px4_sitl gz_standard_vtol
  ```
* **터미널 3**: ROS 2 토픽 확인

  ```bash
  source /opt/ros/humble/setup.bash
  source ~/workspace/ros2/px4_ros2_ws/install/setup.bash
  ros2 topic list | grep fmu
  ```

Windows에서 QGroundControl 실행 후 **Takeoff**, **Transition** 버튼으로 VTOL 동작을 확인하세요!

```
