아래는 Ubuntu 24.04 환경에 맞게 ROS2와 Gazebo Harmonic, 그리고 관련 데모 및 빌드 절차를 모두 반영한 MD 파일의 최신 버전입니다.  
주요 변경 사항은 다음과 같습니다:

- **Ubuntu 버전:** 22.04 → 24.04  
- **ROS2 배포판:** Humble → Iron  
- **APT 설치 패키지 명칭:** “ros-humble-…” → “ros-iron-…”  
- **Repos 파일 내 gz_ros2_control(및 ROS 연동 패키지)의 버전:** 가능하다면 “humble” 대신 “iron”을 사용 (해당 브랜치가 존재하면)

아래 내용을 그대로 사용하시면 됩니다.

---

# 🚀 WSL2 Ubuntu 24.04에서 Space ROS Canadarm 시뮬레이션 구축 가이드 (Gazebo Harmonic 버전)

**환경:** Windows 10/11 (WSL2), Ubuntu 24.04  
**목표:** Docker 없이 Space ROS Canadarm 시뮬레이션 환경 구축 (Gazebo Harmonic 사용)  
**비고:** 이 가이드는 Open Robotics의 Dockerfile에서 사용한 모든 설정과 빌드 과정을 그대로 재현합니다.  
**참고:** 컴퓨터 사양에 따라 빌드 옵션을 조정할 수 있도록 예시도 포함되어 있습니다.

---

## 📌 목차
1. [WSL2 환경 설정](#1-wsl2-환경-설정)
2. [ROS2 Iron 설치](#2-ros2-iron-설치)
3. [필수 패키지 설치](#3-필수-패키지-설치)
4. [Gazebo Harmonic 설치 및 ROS 관련 패키지 설치](#4-gazebo-harmonic-설치-및-ros-관련-패키지-설치)
   - 4.1. **APT 설치 방식**
   - 4.2. **소스 빌드 방식 (선택 사항)**
5. [워크스페이스 소스 클론 및 빌드 준비](#5-워크스페이스-소스-클론-및-빌드-준비)
   - 5.1. Space ROS 소스 클론 (simulation, demos)
6. [데모 의존성 소스 코드 가져오기 (통신 모듈 포함 & repos 파일 자동 생성)](#6-데모-의존성-소스-코드-가져오기-통신-모듈-포함--repos-파일-자동-생성)
7. [의존성 설치 및 전체 빌드 (warehouse_ros_mongo 건너뛰기)](#7-의존성-설치-및-전체-빌드-warehouse_ros_mongo-건너뛰기)
8. [환경 변수 및 추가 설정 적용](#8-환경-변수-및-추가-설정-적용)
9. [X서버(VcXsrv) 및 GUI 설정](#9-x서버vcxsrv-및-gui-설정)
10. [OpenGL 문제 해결 (소프트웨어 렌더링)](#10-opengl-문제-해결-소프트웨어-렌더링)
11. [ROS 작업공간 실행 및 Canadarm 시뮬레이션 실행](#11-ros-작업공간-실행-및-canadarm-시뮬레이션-실행)
12. [GPU 사용 시 사용자 그룹 추가](#12-gpu-사용-시-사용자-그룹-추가)
13. [자주 발생하는 오류 및 해결책](#13-자주-발생하는-오류-및-해결책)
14. [설치 프로그램 및 명령어의 역할](#14-설치-프로그램-및-명령어의-역할)
15. [마무리 및 추가 자료](#15-마무리-및-추가-자료)

---

## 1. WSL2 환경 설정

### 1.1 Ubuntu 24.04 설치  
Windows CMD/PowerShell에서:
```powershell
wsl --install -d Ubuntu-24.04
```

### 1.2 WSL2 자원 제한 (선택 권장)  
`C:\Users\<사용자명>\.wslconfig` 파일에 다음 내용을 작성:
```ini
[wsl2]
memory=8GB
processors=4
swap=2GB
```
적용 후:
```cmd
wsl --shutdown
```
(필요 시 빠른 초기화를 위해)
```bash
wsl --terminate Ubuntu-24.04
wsl --unregister Ubuntu-24.04
wsl --install -d Ubuntu-24.04
```
재부팅 후 Ubuntu 사용자명 및 암호 설정.

---

## 2. ROS2 Iron 설치

### 2.1 Locale 설정
```bash
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### 2.2 ROS2 저장소 키 및 저장소 추가
```bash
sudo apt install -y curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
*참고:* Ubuntu 24.04에서는 ROS2 Iron이 공식 지원됩니다.

### 2.3 ROS2 Iron 설치
```bash
sudo apt update
sudo apt install -y ros-iron-desktop python3-colcon-common-extensions python3-rosdep python3-vcstool
```

### 2.4 추가 ROS 개발 도구 (선택)
```bash
sudo apt install -y python3-pip python3-colcon-mixin python3-flake8 python3-pytest-cov python3-rosinstall-generator ros-iron-ament-* ros-iron-ros-testing ros-iron-eigen3-cmake-module
```

### 2.5 ROS2 환경 설정
```bash
echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo rosdep init || true
rosdep update
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update
```

### 2.6 설치 확인
```bash
ros2 --help
ros2 topic list
printenv ROS_DISTRO  # "iron" 출력 확인
```

---

## 3. 필수 패키지 설치
```bash
sudo apt install -y ros-iron-ros-gz ros-iron-moveit ros-iron-ros2-control ros-iron-ros2-controllers ros-iron-joint-state-publisher ros-iron-xacro ros-iron-ros-ign-bridge libasio-dev git-lfs
```

---

## 4. Gazebo Harmonic 설치 및 ROS 관련 패키지 설치

### 4.1 APT 설치 방식  
아래 명령어로 공식 패키지 저장소를 이용하여 Gazebo Harmonic 및 ROS 연동 패키지들을 설치합니다.
```bash
sudo apt install -y gz-harmonic ros-iron-ros-gz ros-iron-gz-ros2-control ros-iron-joint-state-publisher-gui ros-iron-xacro ros-iron-robot-state-publisher ros-iron-controller-manager
```
*참고:* 설치 후 `gz sim --version` 명령어로 Gazebo Harmonic 버전을 확인하세요.

### 4.2 소스 빌드 방식 (선택 사항)  
APT 패키지로 제공되지 않거나 최신 소스를 사용하고자 하는 경우, OSRF Gazebo Harmonic 소스를 직접 빌드할 수 있습니다.
1. **OSRF 저장소 추가 (APT 방식과 동일하게)**
   ```bash
   sudo apt install -y wget lsb-release gnupg
   sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/gazebo-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list
   sudo apt update
   ```
2. **소스 다운로드 및 빌드 (예시)**
   ```bash
   cd ~
   git clone https://github.com/gazebosim/gz-sim.git
   cd gz-sim
   # Harmonic 버전 태그(예: harmonic)를 체크아웃 (태그명은 공식 문서를 참고)
   git checkout harmonic
   mkdir build && cd build
   cmake .. -DCMAKE_BUILD_TYPE=Release
   make -j$(nproc)
   sudo make install
   ```
   *참고:* 이 방법은 APT 설치 대신 최신 소스나 커스터마이징이 필요한 경우에 사용합니다.

---

## 5. 워크스페이스 소스 클론 및 빌드 준비

### 5.1 Space ROS 소스 클론 (simulation, demos)
```bash
mkdir -p ~/space_ros_ws/src && cd ~/space_ros_ws/src
git lfs install
git clone https://github.com/space-ros/simulation.git
git clone https://github.com/space-ros/demos.git
```
*설명:*  
- **simulation:** Canadarm URDF, 모델, Gazebo 월드 파일  
- **demos:** Canadarm 시뮬레이션 데모 코드 및 launch 파일

---

## 6. 데모 의존성 소스 코드 가져오기 (통신 모듈 포함 & repos 파일 자동 생성)
1. **repos 파일 생성 (자동 복붙)**
   ```bash
   cat << 'EOF' > ~/space_ros_ws/src/demo_manual_pkgs.repos
   repositories:
     demos:
       type: git
       url: https://github.com/space-ros/demos.git
       version: main
     gz_ros2_control:
       type: git
       url: https://github.com/ros-controls/gz_ros2_control.git
       version: iron
     qt_gui_core:
       type: git
       url: https://github.com/ros-visualization/qt_gui_core.git
       version: iron
     ros2_controllers:
       type: git
       url: https://github.com/tonylitianyu/ros2_controllers.git
       version: effort_group_position_controller_2
     actuator_msgs:
       type: git
       url: https://github.com/rudislabs/actuator_msgs.git
       version: main
     ros_gz:
       type: git
       url: https://github.com/gazebosim/ros_gz.git
       version: iron
     simulation:
       type: git
       url: https://github.com/space-ros/simulation.git
       version: main
     vision_msgs:
       type: git
       url: https://github.com/ros-perception/vision_msgs.git
       version: ros2
     gps_msgs:
       type: git
       url: https://github.com/swri-robotics/gps_umd.git
       path: gps_msgs
       version: 113782d
   EOF
   ```
2. **소스 코드 내려받기**
   ```bash
   cd ~/space_ros_ws/src
   vcs import < ~/space_ros_ws/src/demo_manual_pkgs.repos
   ```
*설명:*  
이 과정에서 데모 실행에 필요한 통신 모듈 및 기타 의존 소스 코드가 중복 없이 내려받아집니다.

---

## 7. 의존성 설치 및 전체 빌드 (warehouse_ros_mongo 건너뛰기)
```bash
cd ~/space_ros_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y --skip-keys warehouse_ros_mongo
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```
*설명:*  
- 위 명령어는 모든 소스(시뮬레이션, 데모, 기타 의존성)를 빌드합니다.  
- 만약 컴퓨터 사양이 낮다면 아래와 같이 빌드 옵션을 조정할 수 있습니다:
  ```bash
  MAKEFLAGS="-j1" colcon build --symlink-install --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-O1"
  ```
  *설명:* 이 옵션은 빌드 시 동시에 하나의 작업만 수행하여 메모리 사용량과 CPU 부하를 줄입니다.

---

## 8. 환경 변수 및 추가 설정 적용
```bash
echo "source ~/space_ros_ws/install/setup.bash" >> ~/.bashrc
echo "export GZ_SIM_RESOURCE_PATH=\$GZ_SIM_RESOURCE_PATH:~/space_ros_ws/install/simulation/share/simulation/models" >> ~/.bashrc
echo "export GZ_SIM_SYSTEM_PLUGIN_PATH=~/space_ros_ws/install/lib" >> ~/.bashrc
source ~/.bashrc
```

---

## 9. X서버(VcXsrv) 및 GUI 설정
1. **VcXsrv 다운로드:**  
   [VcXsrv 다운로드](https://sourceforge.net/projects/vcxsrv/)
2. **XLaunch 설정:**  
   - Multiple windows 선택  
   - Display Number: 0  
   - Start no client 선택  
   - Disable access control 체크
3. **WSL2에서 DISPLAY 환경 변수 설정:**
   ```bash
   echo "export DISPLAY=$(grep nameserver /etc/resolv.conf | awk '{print $2}'):0" >> ~/.bashrc
   source ~/.bashrc
   ```
4. **테스트:**
   ```bash
   sudo apt install -y x11-apps
   xeyes
   ```
*설명:* xeyes 창이 뜨면 X 서버 연결이 정상입니다. (DISPLAY는 `:0`이어야 합니다)

---

## 10. OpenGL 문제 해결 (소프트웨어 렌더링)
```bash
echo "export LIBGL_ALWAYS_SOFTWARE=1" >> ~/.bashrc
source ~/.bashrc
```
*설명:* GPU 대신 CPU 기반 소프트웨어 렌더링을 강제하여 OpenGL 오류를 회피합니다. (성능 저하 가능)

---

## 11. ROS 작업공간 실행 및 Canadarm 시뮬레이션 실행

1. **Canadarm 시뮬레이션 실행:**
   ```bash
   ros2 launch canadarm canadarm.launch.py
   ```
   또는 (MoveIt 데모 실행)
   ```bash
   ros2 launch canadarm_moveit_config demo.launch.py
   ```
*설명:* 정상 실행 시, Gazebo 창에 로봇 모델이 나타납니다.

---

## 12. GPU 사용 시 사용자 그룹 추가
```bash
sudo usermod -aG render $USER
```
*참고:* 재부팅 후 적용

---

## 13. 자주 발생하는 오류 및 해결책

| **오류 메시지**                                      | **원인**                                         | **해결책**                                                            |
|------------------------------------------------------|--------------------------------------------------|-----------------------------------------------------------------------|
| `rosdep init` 이미 초기화됨                           | rosdep이 이미 초기화됨                             | 무시하고 `rosdep update` 실행                                          |
| `colcon mixin add` 오류                                | colcon-mixin 미설치                                | `sudo apt install python3-colcon-mixin -y` 후 재시도                    |
| OpenGL/GLX 오류 ("Failed to create OpenGL context")    | GPU 가속/X 서버 설정 문제                           | `export LIBGL_ALWAYS_SOFTWARE=1`로 설정                                |
| Gazebo 창이 검은 화면에서 바로 꺼짐                  | X서버 설정 불량 또는 OpenGL 오류                    | VcXsrv "Disable access control" 체크, DISPLAY=:0, LIBGL_ALWAYS_SOFTWARE 적용 |
| `ros2 control load_controller` 서비스 오류          | Gazebo 실행 또는 컨트롤러 초기화 타이밍 문제          | Gazebo 및 컨트롤러 매니저 초기화 상태 확인 후 재시도                      |
| Duplicate package "warehouse_ros_mongo" 오류        | apt 패키지와 소스 클론(또는 repos 파일) 중복         | 소스 빌드를 위해 apt 패키지 제거(또는 repos 파일을 통해 한 번만 내려받기)   |

---

## 14. 설치 프로그램 및 명령어의 역할

### WSL2 및 Ubuntu 24.04
- **역할:** Windows에서 리눅스 환경 제공 (ROS2, Gazebo 등 실행)

### ROS2 Iron
- **역할:** 로봇 소프트웨어 개발 핵심 프레임워크  
- **설치 이유:**  
  - `ros-iron-desktop`: 기본 GUI 도구 및 기능 포함  
  - `colcon`: 다중 ROS 패키지 빌드  
  - `rosdep`: 의존성 자동 설치

### Gazebo Harmonic
- **역할:** 가상 시뮬레이션 환경 제공 (최신 LTS 버전)  
- **설치 이유:** ROS2와 연동하여 로봇 동작 테스트 및 최신 OGRE2 기반 렌더링 제공  
- **설치 방법:**  
  - **APT 방식:** 공식 패키지 저장소를 통해 설치  
  - **소스 빌드 방식 (선택 사항):** 최신 소스나 커스터마이징이 필요할 경우 직접 빌드하여 설치

### X서버 (VcXsrv)
- **역할:** WSL2에서 GUI 애플리케이션 화면 표시  
- **설치 이유:** Gazebo, Rviz 등 실행

### OpenGL 관련 환경 변수
- **LIBGL_ALWAYS_SOFTWARE:** 소프트웨어 렌더링 강제 (오류 회피)  
- **DISPLAY:** X 서버 연결 설정

### 통신 모듈 및 데모 의존성
- **역할:**  
  - `gz_ros2_control`, `ros_gz`, `ros2_controllers`, `actuator_msgs` 등은 시뮬레이션 통신 및 제어에 필수  
  - demo_manual_pkgs.repos를 통해 자동으로 내려받아 빌드

### MongoDB C++ 드라이버 및 EP_mnmlstc_core 문제 처리
- **참고:** 이번 가이드에서는 몽고 관련 항목은 필요 없으므로 제외합니다.

---

## 15. 마무리 및 추가 자료

모든 설정과 빌드를 완료하면, WSL2 환경에서 ROS2 Iron, Gazebo Harmonic, 그리고 Canadarm 시뮬레이션을 안정적으로 실행할 수 있습니다.  
이 가이드는 Open Robotics의 Dockerfile에서 수행된 모든 단계(소스 클론, 의존성 설치, 데모 소스 내려받기, 빌드, 환경 변수 및 추가 설정, 사용자 그룹 추가 등)를 로컬에서도 동일하게 재현할 수 있도록 구성되었습니다.

**추가 자료:**
- [ROS2 Iron 공식 문서](https://docs.ros.org/en/iron/)
- [Gazebo Harmonic 공식 문서](https://gazebosim.org/docs/harmonic)
- [Space ROS 공식 리포지토리](https://github.com/space-ros)
- [VcXsrv 다운로드 및 설정 안내](https://sourceforge.net/projects/vcxsrv/)

---

이상으로 Ubuntu 24.04 기준으로 업데이트된 가이드를 마무리합니다.  
이 내용대로 진행하면 apt와 소스 빌드 방식을 모두 활용하여 데모 실행에 필요한 모든 소스들이 내려받아지고, ROS2 Iron과 Gazebo Harmonic 환경이 올바르게 구축됩니다.  
추가 수정이나 문의 사항이 있으면 말씀해 주세요!
