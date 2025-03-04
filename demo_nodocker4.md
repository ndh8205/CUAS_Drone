# 🚀 WSL2 Ubuntu 22.04에서 Space ROS Canadarm 시뮬레이션 구축 가이드 (최종 완성본)

**환경:** Windows 10/11 (WSL2), Ubuntu 22.04  
**목표:** Docker 없이 Space ROS Canadarm 시뮬레이션 구현  
**참고:** 도커 이미지에서는 RTPS/DDS, ros2_control, 통신 모듈, MongoDB C++ 드라이버 빌드 및 EP_mnmlstc_core 관련 문제가 미리 패치되어 자동으로 처리됩니다.  
이 가이드는 도커 이미지에서 수행한 모든 수정 사항을 로컬에서도 동일하게 재현할 수 있도록 업데이트되었습니다.

---

## 📌 목차
1. [WSL2 환경 설정](#1-wsl2-환경-설정)
2. [ROS2 Humble 설치](#2-ros2-humble-설치)
3. [필수 패키지 설치](#3-필수-패키지-설치)
4. [Space ROS 패키지 클론](#4-space-ros-패키지-클론)
5. [워크스페이스 빌드 및 데모 의존성 가져오기](#5-워크스페이스-빌드-및-데모-의존성-가져오기)
   - (5.1) Space ROS 패키지 클론 (simulation, demos)
   - (5.2) warehouse_ros_mongo 소스 클론
   - (5.3) MongoDB C++ 드라이버 설치 (추가 단계)
   - (5.4) 데모 의존성 소스 코드 가져오기 (통신 모듈 포함 & repos 파일 자동 생성)
   - (5.5) 의존성 설치 및 전체 빌드
6. [환경 변수 및 추가 설정 적용](#6-환경-변수-및-추가-설정-적용)
7. [X서버(VcXsrv) 및 GUI 설정](#7-x서버vcxsrv-및-gui-설정)
8. [OpenGL 문제 해결 및 환경 변수 설정](#8-opengl-문제-해결-및-환경-변수-설정)
9. [ROS 작업공간 실행 및 Canadarm 시뮬레이션 실행](#9-ros-작업공간-실행-및-canadarm-시뮬레이션-실행)
10. [GPU 사용 시 사용자 그룹 추가](#10-gpu-사용-시-사용자-그룹-추가)
11. [자주 발생하는 오류 및 빠른 해결책](#11-자주-발생하는-오류-및-빠른-해결책)
12. [설치 프로그램 및 명령어의 역할](#12-설치-프로그램-및-명령어의-역할)
13. [마무리 및 추가 자료](#13-마무리-및-추가-자료)

---

## 1. WSL2 환경 설정

### 1.1 Ubuntu 22.04 설치
Windows CMD/PowerShell에서:
```powershell
wsl --install -d Ubuntu-22.04
```

### 1.2 WSL2 자원 제한 (선택 권장)
`C:\Users\<사용자명>\.wslconfig` 파일에 다음 내용을 작성:
```ini
[wsl2]
memory=8GB
processors=4
swap=2GB
```
적용을 위해:
```cmd
wsl --shutdown
```
필요 시 빠른 초기화를 위해:
```bash
wsl --terminate Ubuntu-22.04
wsl --unregister Ubuntu-22.04
wsl --install -d Ubuntu-22.04
```
재부팅 후 Ubuntu 사용자명과 암호 설정.

---

## 2. ROS2 Humble 설치

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
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### 2.3 ROS2 Humble 설치
```bash
sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions python3-rosdep python3-vcstool
```

### 2.4 추가 ROS 개발 도구 (선택)
```bash
sudo apt install -y python3-pip python3-colcon-mixin python3-flake8 python3-pytest-cov python3-rosinstall-generator ros-humble-ament-* ros-humble-ros-testing ros-humble-eigen3-cmake-module
```

### 2.5 ROS2 환경 설정
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
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
printenv ROS_DISTRO  # "humble" 출력 확인
```

---

## 3. 필수 패키지 설치
```bash
sudo apt install -y ros-humble-ros-gz ros-humble-moveit ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-joint-state-publisher ros-humble-xacro ros-humble-ros-ign-bridge libasio-dev git-lfs
```

---

## 4. Gazebo(Ignition) 및 ROS 관련 패키지 설치

**목적:**  
Gazebo는 로봇 시뮬레이션 환경으로, ROS와 연동하기 위해 필요한 패키지들을 설치합니다.
```bash
sudo apt install -y ros-humble-ros-gz ros-humble-ign-ros2-control ros-humble-joint-state-publisher-gui ros-humble-xacro ros-humble-robot-state-publisher ros-humble-controller-manager
```
그리고 MoveIt 데이터베이스 백엔드로 사용할 **warehouse_ros_sqlite**도 설치합니다:
```bash
sudo apt install -y ros-humble-warehouse-ros-sqlite
```

---

## 5. 워크스페이스 빌드 및 데모 의존성 가져오기

### 5.1 Space ROS 패키지 클론
```bash
mkdir -p ~/space_ros_ws/src && cd ~/space_ros_ws/src
git lfs install
git clone https://github.com/space-ros/simulation.git
git clone https://github.com/space-ros/demos.git
```
- **설명:**  
  - `simulation`: Canadarm URDF, 모델, Gazebo 월드 파일 포함  
  - `demos`: Canadarm 시뮬레이션 실행 데모 코드 및 launch 파일 포함

---

### 5.2 warehouse_ros_mongo 소스 클론
```bash
cd ~/space_ros_ws/src
git clone -b ros2 https://github.com/ros-planning/warehouse_ros_mongo.git
```
- **중요:** 반드시 ROS2용 브랜치(`ros2`) 사용

---

### 5.3 MongoDB C++ 드라이버 설치 (추가 단계)
`warehouse_ros_mongo`가 MongoDB와 연동되려면 MongoDB C++ 드라이버가 필요합니다.

1. **필수 의존성 설치**
   ```bash
   sudo apt install -y libssl-dev build-essential devscripts debian-keyring fakeroot debhelper cmake libboost-dev libsasl2-dev libicu-dev libzstd-dev doxygen
   ```
2. **libmongoc-dev 설치**  
   (Ubuntu 22.04에서는 제공됨)
   ```bash
   sudo apt install -y libmongoc-dev
   ```
3. **MongoDB C++ 드라이버 다운로드 및 빌드**
   ```bash
   wget https://github.com/mongodb/mongo-cxx-driver/releases/download/r3.6.7/mongo-cxx-driver-r3.6.7.tar.gz
   tar -xzf mongo-cxx-driver-r3.6.7.tar.gz
   cd mongo-cxx-driver-r3.6.7/build
   cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
   sudo cmake --build . --target EP_mnmlstc_core
   cmake --build .
   sudo cmake --build . --target install
   cd ../../
   ```

---

### 5.4 데모 의존성 소스 코드 가져오기 (통신 모듈 포함 & repos 파일 자동 생성)
도커 이미지에서는 `demo_manual_pkgs.repos` 파일을 사용하여 통신 모듈(예: gz_ros2_control, ros_gz, ros2_controllers 등) 및 기타 의존성을 내려받습니다.  
아래 명령어를 터미널에 복사하여 실행하면, 자동으로 repos 파일이 생성됩니다.

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
    version: humble
  qt_gui_core:
    type: git
    url: https://github.com/ros-visualization/qt_gui_core.git
    version: humble
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
    version: humble
  simulation:
    type: git
    url: https://github.com/space-ros/simulation.git
    version: main
  ros-humble-warehouse-ros-mongo:
    type: git
    url: https://github.com/moveit/warehouse_ros_mongo.git
    version: ros2
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

그런 다음 아래 명령어로 repos 파일에 정의된 소스 코드를 내려받습니다.
```bash
cd ~/space_ros_ws/src
vcs import < ~/space_ros_ws/src/demo_manual_pkgs.repos
```
- **설명:**  
  이 과정에서 데모 실행에 필요한 모든 통신 모듈 및 추가 의존 소스 코드가 자동으로 내려받아집니다.

---

### 5.5 의존성 설치 및 전체 빌드
```bash
cd ~/space_ros_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```
- **설명:**  
  위 명령어로 모든 소스 코드(시뮬레이션, 데모, warehouse_ros_mongo 등)를 빌드합니다.

---

## 6. 환경 변수 및 추가 설정 적용
아래 환경 변수를 추가하여, ROS2가 빌드된 패키지와 시뮬레이션 모델을 올바르게 인식하도록 합니다.
```bash
echo "source ~/space_ros_ws/install/setup.bash" >> ~/.bashrc
echo "export IGN_GAZEBO_RESOURCE_PATH=\$IGN_GAZEBO_RESOURCE_PATH:~/space_ros_ws/install/simulation/share/simulation/models" >> ~/.bashrc
source ~/.bashrc
```

---

## 7. X서버(VcXsrv) 및 GUI 설정

**목적:** WSL2에서 실행되는 GUI 애플리케이션(예: Gazebo, Rviz)을 Windows에서 볼 수 있도록 합니다.

1. **VcXsrv 다운로드:**  
   [VcXsrv 다운로드](https://sourceforge.net/projects/vcxsrv/)
2. **XLaunch 설정:**  
   - **Multiple windows** 선택  
   - **Display Number:** 0  
   - **Start no client** 선택  
   - **Disable access control** 옵션 체크
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
   - **설명:** xeyes 창이 뜨면 X 서버 연결이 정상입니다.  
   - **주의:** DISPLAY는 `:0`로 설정되어야 합니다.

---

## 8. OpenGL 문제 해결 및 환경 변수 설정

**문제:** WSL2에서 GPU 하드웨어 가속이 불안정하여 Gazebo가 OpenGL 관련 오류로 종료될 수 있습니다.  
**해결 방법 (권장 - 소프트웨어 렌더링):**
```bash
echo "export LIBGL_ALWAYS_SOFTWARE=1" >> ~/.bashrc
source ~/.bashrc
```
- **설명:** GPU 대신 CPU 기반 소프트웨어 렌더링을 강제하여 OpenGL 오류를 회피합니다. (단, 성능은 다소 저하될 수 있음)

---

## 9. ROS 작업공간 실행 및 Canadarm 시뮬레이션 실행

1. **Workspace 빌드 (재확인):**
   ```bash
   cd ~/space_ros_ws
   colcon build --symlink-install --cmake-clean-cache
   source install/setup.bash
   ```
2. **Canadarm 시뮬레이션 실행:**
   ```bash
   ros2 launch canadarm canadarm.launch.py robot_description_file:=$(ros2 pkg prefix simulation)/share/simulation/models/canadarm/urdf/SSRMS_Canadarm2.urdf.xacro
   ```
   또는 (MoveIt 데모 실행)
   ```bash
   ros2 launch canadarm_moveit_config demo.launch.py
   ```
   - **설명:** 위 명령어는 Canadarm 시뮬레이션을 실행하며, 외부 URDF 파일을 불러옵니다. 정상 실행 시 Gazebo 창에 로봇 모델이 나타납니다.

---

## 10. GPU 사용 시 사용자 그룹 추가

GPU를 활용하는 경우, 렌더링 장치 접근을 위해 사용자를 render 그룹에 추가합니다:
```bash
sudo usermod -aG render $USER
```
*참고: 재부팅 후 적용됩니다.*

---

## 11. 자주 발생하는 오류 및 빠른 해결책

| **오류 메시지**                                   | **원인**                                     | **해결책**                                                            |
|---------------------------------------------------|----------------------------------------------|-----------------------------------------------------------------------|
| `rosdep init` 이미 초기화됨                        | rosdep이 이미 초기화됨                        | 무시하고 `rosdep update`만 실행                                        |
| `colcon mixin add` 오류                             | colcon-mixin 패키지가 설치되지 않음             | `sudo apt install python3-colcon-mixin -y` 후 재시도                     |
| OpenGL/GLX 오류 ("Failed to create OpenGL context")   | GPU 가속 문제, X 서버 설정 문제               | `export LIBGL_ALWAYS_SOFTWARE=1`로 설정 (또는 GPU 가속 환경 구성)         |
| Gazebo 창이 검은 화면에서 바로 꺼짐                 | X서버 설정 불량 또는 OpenGL 오류               | VcXsrv 설정에서 "Disable access control" 체크, DISPLAY=:0, LIBGL_ALWAYS_SOFTWARE 적용 |
| `ros2 control load_controller` 서비스 오류         | Gazebo 실행 문제 또는 컨트롤러 초기화 타이밍 문제  | Gazebo와 컨트롤러 매니저 초기화 상태 확인 후 재시도                        |

---

## 12. 설치 프로그램 및 명령어의 역할

### WSL2 및 Ubuntu 22.04
- **역할:** Windows에서 리눅스 환경 제공 (ROS2, Gazebo 등 실행)

### ROS2 Humble
- **역할:** 로봇 소프트웨어 개발 핵심 프레임워크  
- **설치 이유:**  
  - `ros-humble-desktop`: 기본 GUI 도구 및 기능 포함  
  - `colcon`: 다중 ROS 패키지 빌드  
  - `rosdep`: 의존성 자동 설치

### Gazebo (Ignition)
- **역할:** 가상 시뮬레이션 환경 제공  
- **설치 이유:** ROS2와 연동하여 로봇 동작 테스트

### X서버 (VcXsrv)
- **역할:** WSL2에서 GUI 애플리케이션 화면 표시  
- **설치 이유:** Gazebo, Rviz 등 실행

### OpenGL 관련 환경 변수
- **LIBGL_ALWAYS_SOFTWARE:** 소프트웨어 렌더링 강제 (오류 회피)  
- **DISPLAY:** X 서버 연결 설정

### 통신 모듈 및 데모 의존성
- **역할:**  
  - `gz_ros2_control`, `ros_gz`, `ros2_controllers`, `actuator_msgs` 등은 시뮬레이션의 통신 및 제어에 필수  
  - demo_manual_pkgs.repos (또는 위에 생성한 repos 파일)을 통해 자동으로 내려받아 빌드

### MongoDB C++ 드라이버 및 EP_mnmlstc_core 문제 해결
- **문제:** MongoDB C++ 드라이버 빌드시 EP_mnmlstc_core 설치 단계에서 오류 발생  
- **해결:**  
  - Ubuntu 22.04에서는 libmongoc-dev를 설치하여 필요한 CMake 구성 파일을 확보  
  - 도커 이미지에서는 해당 문제를 패치(또는 install 단계 무시)하여 해결되어 있음  
  - 로컬에서는 EP_mnmlstc_core의 CMakeLists.txt에 dummy install 규칙을 추가하거나, install 오류를 무시하도록 처리

---

## 13. 마무리 및 추가 자료

모든 설정과 빌드를 완료하면, WSL2 환경에서 ROS2 Humble, Gazebo, 그리고 Canadarm 시뮬레이션을 안정적으로 실행할 수 있습니다.  
이 가이드는 도커 이미지 빌드 스크립트에서 수행된 모든 과정(통신 모듈, MongoDB C++ 드라이버 빌드, EP_mnmlstc_core 문제 처리, 사용자 그룹 설정, 자동 repos 파일 생성 등)을 누락 없이 반영하여 업데이트되었습니다.

**추가 자료:**
- [ROS2 Humble 공식 문서](https://docs.ros.org/en/humble/)
- [Space ROS 공식 리포지토리](https://github.com/space-ros)
- [VcXsrv 다운로드 및 설정 안내](https://sourceforge.net/projects/vcxsrv/)
