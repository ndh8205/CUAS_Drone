# 🚀 WSL2 Ubuntu 22.04에서 Space ROS Canadarm 시뮬레이션 구축 가이드 (Gazebo Harmonic)

**환경:** Windows 10/11 (WSL2), Ubuntu 22.04  
**목표:** Docker 없이 Space ROS Canadarm 시뮬레이션 환경 구축 (Gazebo Harmonic 사용)  
**비고:** 이 가이드는 Open Robotics의 Dockerfile에서 사용한 모든 설정과 빌드 과정을 그대로 재현합니다.

---

## 📌 목차
1. [WSL2 환경 설정](#1-wsl2-환경-설정)
2. [ROS2 Humble 설치](#2-ros2-humble-설치)
3. [필수 패키지 설치](#3-필수-패키지-설치)
4. [Gazebo Harmonic 및 ROS 관련 패키지 설치](#4-gazebo-harmonic-및-ros-관련-패키지-설치)
5. [워크스페이스 소스 클론 및 빌드 준비](#5-워크스페이스-소스-클론-및-빌드-준비)
   - 5.1 Space ROS 소스 클론 (simulation, demos)
6. [의존성 설치 및 전체 빌드](#6-의존성-설치-및-전체-빌드)
7. [환경 변수 및 추가 설정 적용](#7-환경-변수-및-추가-설정-적용)
8. [X서버(VcXsrv) 및 GUI 설정](#8-x서버vcxsrv-및-gui-설정)
9. [OpenGL 문제 해결 (소프트웨어 렌더링)](#9-opengl-문제-해결)
10. [ROS 작업공간 실행 및 Canadarm 시뮬레이션 실행](#10-ros-작업공간-및-canadarm-시뮬레이션-실행)
11. [GPU 사용 시 사용자 그룹 추가](#11-gpu-사용-시-사용자-그룹-추가)
12. [자주 발생하는 오류 및 해결책](#12-자주-발생하는-오류-및-해결책)
13. [설치 프로그램 및 명령어의 역할](#13-설치-프로그램-및-명령어의-역할)
14. [마무리 및 추가 자료](#14-마무리-및-추가-자료)

---

## 📌 1. WSL2 환경 설정
### 1.1 Ubuntu 22.04 설치
```powershell
wsl --install -d Ubuntu-22.04
```

---

## 📌 2. ROS2 Humble 설치
기존 가이드와 동일 (생략)

---

## 📌 3. 필수 패키지 설치
```bash
sudo apt install -y python3-colcon-common-extensions ros-humble-moveit ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-joint-state-publisher ros-humble-xacro ros-humble-robot-state-publisher ros-humble-controller-manager libasio-dev git-lfs
```

---

## 📌 4. Gazebo Harmonic 및 ROS 관련 패키지 설치

### 4.1 OSRF Gazebo 패키지 저장소 추가
```bash
sudo apt install -y wget lsb-release gnupg
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list
```

### Gazebo Harmonic 설치
```bash
sudo apt update
sudo apt install -y gz-harmonic ros-humble-ros-gz ros-humble-gz-ros2-control
```

---

## 📌 5. 워크스페이스 소스 클론 및 빌드 준비

### 5.1 Space ROS 소스 클론 (simulation, demos)
```bash
mkdir -p ~/space_ros_ws/src && cd ~/space_ros_ws/src
git lfs install
git clone https://github.com/space-ros/simulation.git
git clone https://github.com/space-ros/demos.git
```
*설명:*  
- **simulation:** Canadarm 모델, Gazebo 월드 파일  
- **demos:** Canadarm 시뮬레이션 데모 코드 및 launch 파일

---

## 📌 6. 의존성 설치 및 전체 빌드
```bash
cd ~/space_ros_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y --skip-keys warehouse_ros_mongo
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

(성능 제한 빌드)
```bash
MAKEFLAGS="-j1" colcon build --symlink-install --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-O1" --allow-overriding joint_trajectory_controller qt_gui qt_gui_cpp
```

---

## 📌 7. 환경 변수 및 추가 설정 적용
```bash
echo "source ~/space_ros_ws/install/setup.bash" >> ~/.bashrc
echo "export GZ_SIM_RESOURCE_PATH=\$GZ_SIM_RESOURCE_PATH:~/space_ros_ws/install/simulation/share/simulation/models" >> ~/.bashrc
source ~/.bashrc
```

---

## 📌 8. X서버(VcXsrv) 및 GUI 설정
- [VcXsrv 설치링크](https://sourceforge.net/projects/vcxsrv/)
- DISPLAY 환경변수 설정:
```bash
echo "export DISPLAY=$(grep nameserver /etc/resolv.conf | awk '{print $2}'):0" >> ~/.bashrc
source ~/.bashrc
```

---

## 📌 9. OpenGL 문제 해결 (소프트웨어 렌더링)
```bash
echo "export LIBGL_ALWAYS_SOFTWARE=1" >> ~/.bashrc
source ~/.bashrc
```

---

## 📌 10. ROS 작업공간 및 Canadarm 시뮬레이션 실행
### 10.1 시뮬레이션 실행
```bash
ros2 launch canadarm canadarm.launch.py
# MoveIt 연동
ros2 launch canadarm_moveit_config demo.launch.py
```

---

## 📌 11. GPU 사용 시 사용자 그룹 추가
```bash
sudo usermod -aG render $USER
```

---

## 📌 12. 자주 발생하는 오류 및 해결책
| 오류 | 원인 | 해결책 |
|---|---|---|
|`rosdep init` 실패| 이미 초기화됨| `rosdep update`만 수행|
|OpenGL 오류|GPU 가속 불가|LIBGL_ALWAYS_SOFTWARE=1 설정|
|Gazebo 실행 실패| DISPLAY 설정 문제, GPU 가속 문제|VcXsrv 옵션 설정 확인, LIBGL_ALWAYS_SOFTWARE|

---

## 📌 13. 설치 프로그램 및 명령어 역할
- **Gazebo Harmonic**: 시뮬레이션 환경 (최신 LTS)
- **ROS2 Humble**: 로봇 운영체제 핵심 프레임워크
- **VcXsrv**: GUI 지원을 위한 X 서버

---

## 📌 14. 마무리 및 추가 자료
- [ROS2 Humble 공식 문서](https://docs.ros.org/en/humble/)
- [Gazebo Harmonic 공식 문서](https://gazebosim.org/docs/harmonic)
- [Space ROS GitHub](https://github.com/space-ros)
