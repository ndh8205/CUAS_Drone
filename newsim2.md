# 🚀 WSL2 Ubuntu 22.04에서 Space ROS Canadarm 시뮬레이션 구축 가이드

**환경:** Windows 10/11 (WSL2), Ubuntu 22.04  
**목표:** Docker 없이 Space ROS Canadarm 시뮬레이션 환경 구축  
**비고:** 이 가이드는 Open Robotics의 Dockerfile에서 사용한 모든 설정과 빌드 과정을 그대로 재현합니다.

---

## 📌 목차
1. [WSL2 환경 설정](#1-wsl2-환경-설정)
2. [ROS2 Humble 설치](#2-ros2-humble-설치)
3. [필수 패키지 설치](#3-필수-패키지-설치)
4. [Gazebo Harmonic 및 ROS 관련 패키지 설치](#4-gazebo-harmonic-및-ros-관련-패키지-설치)
5. [워크스페이스 소스 클론 및 빌드 준비](#5-워크스페이스-소스-클론-및-빌드-준비)
   - 5.1. Space ROS 소스 클론 (simulation, demos)
6. [의존성 설치 및 전체 빌드](#6-의존성-설치-및-전체-빌드)
7. [환경 변수 및 추가 설정 적용](#7-환경-변수-및-추가-설정-적용)
8. [X서버(VcXsrv) 및 GUI 설정](#8-x서버vcxsrv-및-gui-설정)
9. [OpenGL 문제 해결 (소프트웨어 렌더링)](#9-opengl-문제-해결)
10. [ROS 작업공간 실행 및 Canadarm 시뮬레이션 실행](#10-ros-작업공간-실행-및-canadarm-시뮬레이션-실행)
11. [GPU 사용 시 사용자 그룹 추가](#11-gpu-사용-시-사용자-그룹-추가)
12. [자주 발생하는 오류 및 해결책](#12-자주-발생하는-오류-및-해결책)
13. [설치 프로그램 및 명령어의 역할](#13-설치-프로그램-및-명령어의-역할)
14. [마무리 및 추가 자료](#14-마무리-및-추가-자료)

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
적용 후:
```cmd
wsl --shutdown
```
(필요 시 빠른 초기화를 위해)
```bash
wsl --terminate Ubuntu-22.04
wsl --unregister Ubuntu-22.04
wsl --install -d Ubuntu-22.04
```
재부팅 후 Ubuntu 사용자명 및 암호 설정.

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

## 4. Gazebo Harmonic 및 ROS 관련 패키지 설치
아래 명령어는 Gazebo Harmonic 설치와 ROS 연동에 필요한 패키지들을 설치합니다.

1. **OSRF 저장소 추가**  
   ```bash
   sudo apt install -y wget lsb-release gnupg
   sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/gazebo-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list
   sudo apt update
   ```
2. **apt 설치 진행**  
   ```bash
   sudo apt install -y gz-harmonic ros-humble-ros-gz ros-humble-gz-ros2-control ros-humble-joint-state-publisher-gui ros-humble-xacro ros-humble-robot-state-publisher ros-humble-controller-manager
   ```

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

## 6. 의존성 설치 및 전체 빌드
```bash
cd ~/space_ros_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y --skip-keys warehouse_ros_mongo
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```
*설명:*  
내 컴퓨터가 너무 구리다면 아래의 옵션을 사용하자

```bash
MAKEFLAGS="-j1" colcon build --symlink-install --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-O1" --allow-overriding joint_trajectory_controller qt_gui qt_gui_cpp
```

*설명:*  
위 명령어는 모든 소스(시뮬레이션, 데모, warehouse_ros_mongo 등)를 빌드하기 전에, warehouse_ros_mongo에 대한 의존성은 별도로 빌드했으므로 건너뛰도록 합니다.
  
> **주의:** Duplicate package 오류가 발생하면, 두 개의 warehouse_ros_mongo 패키지가 소스 내에 중복되어 있는지 확인하고, 하나(예: apt에서 내려받은 버전 또는 repos 파일에서 내려받은 버전)를 삭제하세요.  
> 예시:
> ```bash
> rm -rf ~/space_ros_ws/src/ros-humble-warehouse-ros-mongo
> ```
> 또는 필요에 따라 적절히 정리합니다.

---

## 7. 환경 변수 및 추가 설정 적용
```bash
echo "source ~/space_ros_ws/install/setup.bash" >> ~/.bashrc
echo "export GZ_SIM_RESOURCE_PATH=\$GZ_SIM_RESOURCE_PATH:~/space_ros_ws/install/simulation/share/simulation/models" >> ~/.bashrc
source ~/.bashrc
```

---

## 8. X서버(VcXsrv) 및 GUI 설정
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

## 9. OpenGL 문제 해결 (소프트웨어 렌더링)
```bash
echo "export LIBGL_ALWAYS_SOFTWARE=1" >> ~/.bashrc
source ~/.bashrc
```
*설명:* GPU 대신 CPU 기반 소프트웨어 렌더링을 강제하여 OpenGL 오류를 회피합니다. (성능 저하 가능)

---

## 10. ROS 작업공간 실행 및 Canadarm 시뮬레이션 실행

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

## 11. GPU 사용 시 사용자 그룹 추가
```bash
sudo usermod -aG render $USER
```
*참고:* 재부팅 후 적용

---

## 12. 자주 발생하는 오류 및 해결책

| **오류 메시지**                                      | **원인**                                         | **해결책**                                                            |
|------------------------------------------------------|--------------------------------------------------|-----------------------------------------------------------------------|
| `rosdep init` 이미 초기화됨                           | rosdep이 이미 초기화됨                             | 무시하고 `rosdep update` 실행                                           |
| `colcon mixin add` 오류                                | colcon-mixin 미설치                                | `sudo apt install python3-colcon-mixin -y` 후 재시도                     |
| OpenGL/GLX 오류 ("Failed to create OpenGL context")    | GPU 가속/X 서버 설정 문제                          | `export LIBGL_ALWAYS_SOFTWARE=1`로 설정                                 |
| Gazebo 창이 검은 화면에서 바로 꺼짐                  | X서버 설정 불량 또는 OpenGL 오류                   | VcXsrv "Disable access control" 체크, DISPLAY=:0, LIBGL_ALWAYS_SOFTWARE 적용 |
| `ros2 control load_controller` 서비스 오류          | Gazebo 실행 또는 컨트롤러 초기화 타이밍 문제          | Gazebo 및 컨트롤러 매니저 초기화 상태 확인 후 재시도                       |
| Duplicate package "warehouse_ros_mongo" 오류        | apt 패키지와 소스 클론(또는 repos 파일) 중복         | 소스 빌드를 위해 apt 패키지 제거(또는 repos 파일을 통해 한 번만 내려받기)  |

---

## 13. 설치 프로그램 및 명령어의 역할

### WSL2 및 Ubuntu 22.04
- **역할:** Windows에서 리눅스 환경 제공 (ROS2, Gazebo 등 실행)

### ROS2 Humble
- **역할:** 로봇 소프트웨어 개발 핵심 프레임워크  
- **설치 이유:**  
  - `ros-humble-desktop`: 기본 GUI 도구 및 기능 포함  
  - `colcon`: 다중 ROS 패키지 빌드  
  - `rosdep`: 의존성 자동 설치

### Gazebo Harmonic
- **역할:** 가상 시뮬레이션 환경 제공 (최신 LTS 버전)  
- **설치 이유:** ROS2와 연동하여 로봇 동작 테스트 및 최신 OGRE2 기반 렌더링 제공

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
- **문제:** MongoDB C++ 드라이버 빌드시 EP_mnmlstc_core의 install 단계 오류 발생  
- **해결:**  
  - Ubuntu 22.04에서는 libmongoc-dev 설치로 필요한 CMake 구성 파일을 확보  
  - 도커 이미지에서는 해당 문제를 패치(또는 install 단계 무시)하여 해결됨  
  - 로컬에서는 EP_mnmlstc_core의 CMakeLists.txt에 dummy install 규칙 추가하거나, 중복 패키지 제거를 통해 해결

---

## 14. 마무리 및 추가 자료

모든 설정과 빌드를 완료하면, WSL2 환경에서 ROS2 Humble, Gazebo Harmonic, 그리고 Canadarm 시뮬레이션을 안정적으로 실행할 수 있습니다.  
이 가이드는 Open Robotics의 Dockerfile에서 수행된 모든 단계(소스 클론, 의존성 설치, MongoDB C++ 드라이버 빌드 및 EP_mnmlstc_core 처리, repos 파일 자동 생성, 중복 패키지 제거, 사용자 그룹 추가 등)를 로컬에서도 동일하게 재현할 수 있도록 구성되었습니다.

**추가 자료:**
- [ROS2 Humble 공식 문서](https://docs.ros.org/en/humble/)
- [Space ROS 공식 리포지토리](https://github.com/space-ros)
- [VcXsrv 다운로드 및 설정 안내](https://sourceforge.net/projects/vcxsrv/)




아래는 기존 MD 파일의 전체 내용을 사용자 편의를 최대한 유지하면서, Gazebo Harmonic 설치 파트에 저장소에서 직접 소스 빌드하는 방식(apt 설치 방식과 함께 선택 사항)을 추가한 업데이트 버전입니다. 

---

# 🚀 WSL2 Ubuntu 22.04에서 Space ROS Canadarm 시뮬레이션 구축 가이드 (Gazebo Harmonic 버전)

**환경:** Windows 10/11 (WSL2), Ubuntu 22.04  
**목표:** Docker 없이 Space ROS Canadarm 시뮬레이션 환경 구축 (Gazebo Harmonic 사용)  
**비고:** 이 가이드는 Open Robotics의 Dockerfile에서 사용한 모든 설정과 빌드 과정을 그대로 재현합니다.  
**참고:** 컴퓨터 사양에 따라 빌드 옵션을 조정할 수 있도록 예시도 포함되어 있습니다.

---

## 📌 목차
1. [WSL2 환경 설정](#1-wsl2-환경-설정)
2. [ROS2 Humble 설치](#2-ros2-humble-설치)
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
적용 후:
```cmd
wsl --shutdown
```
(필요 시 빠른 초기화를 위해)
```bash
wsl --terminate Ubuntu-22.04
wsl --unregister Ubuntu-22.04
wsl --install -d Ubuntu-22.04
```
재부팅 후 Ubuntu 사용자명 및 암호 설정.

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

## 4. Gazebo Harmonic 설치 및 ROS 관련 패키지 설치

### 4.1 APT 설치 방식  
아래 명령어로 공식 패키지 저장소를 이용하여 Gazebo Harmonic 및 ROS 연동 패키지들을 설치합니다.
```bash
sudo apt install -y gz-harmonic ros-humble-ros-gz ros-humble-gz-ros2-control ros-humble-joint-state-publisher-gui ros-humble-xacro ros-humble-robot-state-publisher ros-humble-controller-manager
```
*주의:* 설치 후 `gz sim --version`으로 버전을 확인하여 Gazebo Harmonic(예: version 8.9.0)이 설치되었는지 점검합니다.

### 4.2 소스 빌드 방식 (선택 사항)  
APT 패키지가 부족하거나 최신 소스를 사용하고자 하는 경우, 아래와 같이 OSRF Gazebo Harmonic 저장소에서 소스를 가져와 빌드할 수 있습니다.


2. **소스 다운로드 및 빌드 (예시)**  
   Gazebo Harmonic의 소스는 [Gazebo Sim GitHub 저장소](https://github.com/gazebosim/gz-sim)에서 가져올 수 있습니다.
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
   *설명:* 이 방법은 공식 apt 패키지 대신 최신 소스나 커스터마이징이 필요한 경우에 사용합니다. 다만, ROS 연동 부분(ros-humble-gz-ros2-control 등)은 여전히 별도 설치가 필요하므로, 필요에 따라 두 방식을 병행할 수 있습니다.

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
- 만약 내 컴퓨터 사양이 낮다면 아래와 같이 빌드 옵션을 조정할 수 있습니다:
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

### WSL2 및 Ubuntu 22.04
- **역할:** Windows에서 리눅스 환경 제공 (ROS2, Gazebo 등 실행)

### ROS2 Humble
- **역할:** 로봇 소프트웨어 개발 핵심 프레임워크  
- **설치 이유:**  
  - `ros-humble-desktop`: 기본 GUI 도구 및 기능 포함  
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

모든 설정과 빌드를 완료하면, WSL2 환경에서 ROS2 Humble, Gazebo Harmonic, 그리고 Canadarm 시뮬레이션을 안정적으로 실행할 수 있습니다.  
이 가이드는 Open Robotics의 Dockerfile에서 수행된 모든 단계(소스 클론, 의존성 설치, 데모 소스 내려받기, 빌드, 환경 변수 및 추가 설정, 사용자 그룹 추가 등)를 로컬에서도 동일하게 재현할 수 있도록 구성되었습니다.

**추가 자료:**
- [ROS2 Humble 공식 문서](https://docs.ros.org/en/humble/)
- [Gazebo Harmonic 공식 문서](https://gazebosim.org/docs/harmonic)
- [Space ROS 공식 리포지토리](https://github.com/space-ros)
- [VcXsrv 다운로드 및 설정 안내](https://sourceforge.net/projects/vcxsrv/)

---

이상으로 업데이트된 가이드를 마무리합니다.  
이 내용대로 진행하면, 몽고 관련 항목은 제외되고 데모 실행에 필요한 소스들이 자동으로 내려받혀 빌드되며, Gazebo Harmonic은 apt 설치와 함께 필요 시 소스 빌드 방식도 선택할 수 있습니다.  
컴퓨터 사양에 따라 빌드 옵션 조정 예시도 포함되어 있으니, 편의에 맞게 사용하시기 바랍니다.  
추가 수정이나 문의 사항이 있으면 말씀해 주세요!




