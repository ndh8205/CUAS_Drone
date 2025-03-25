# 🚀 WSL2 Ubuntu 22.04에서 Space ROS Canadarm 시뮬레이션 구축 가이드 (Gazebo Garden 사용)

**환경:** Windows 10/11 (WSL2), Ubuntu 22.04  
**목표:** Docker 없이 Space ROS Canadarm 시뮬레이션 환경 구축 (ROS2 Humble, Gazebo Garden 사용)  
**비고:** 이 가이드는 Open Robotics의 Dockerfile에서 사용한 모든 설정과 빌드 과정을 로컬 환경에 재현하는 것을 목표로 합니다.  
**참고:** 컴퓨터 사양에 따라 빌드 옵션을 조정할 수 있도록 예시도 포함되어 있습니다.

---

## 📌 목차
1. [WSL2 환경 설정](#1-wsl2-환경-설정)
2. [ROS2 Humble 설치](#2-ros2-humble-설치)
3. [필수 패키지 설치](#3-필수-패키지-설치)
4. [Gazebo Garden 및 ROS 관련 패키지 설치](#4-gazebo-garden-및-ros-관련-패키지-설치)
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
13. [Gazebo Garden 카메라 뷰 설정 및 최적화](#13-gazebo-garden-카메라-뷰-설정-및-최적화)
14. [모델 분리 및 Include로 사용하기](#14-모델-분리-및-include로-사용하기)
15. [자주 발생하는 오류 및 해결책](#15-자주-발생하는-오류-및-해결책)
16. [설치 프로그램 및 명령어의 역할](#16-설치-프로그램-및-명령어의-역할)
17. [마무리 및 추가 자료](#17-마무리-및-추가-자료)

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
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
*Ubuntu 22.04의 코드네임은 "jammy"입니다.*

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

## 4. Gazebo Garden 및 ROS 관련 패키지 설치

### 4.1 APT 설치 방식  
아래 명령어로 공식 패키지 저장소를 이용하여 Gazebo Garden 및 ROS 연동 패키지들을 설치합니다.

**OSRF 저장소 추가 (APT 방식과 동일하게)**

 ```bash
 sudo apt install -y wget lsb-release gnupg
 sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/gazebo-archive-keyring.gpg
 echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] \
 http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list
 sudo apt update
 ```
   
  ```bash
  sudo apt install -y gz-garden ros-humble-ros-gz ros-humble-ign-ros2-control ros-humble-joint-state-publisher-gui ros-humble-xacro ros-humble-robot-state-publisher ros-humble-controller-manager
  ```
*설치 후 `gz sim --version` 명령어로 Gazebo Garden 버전을 확인하세요.*

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

디렉토리 구조:
```
~/space_ros_ws/
├── src/
│   ├── simulation/
│   │   ├── models/
│   │   │   ├── canadarm/
│   │   │   ├── curiosity_path/
│   │   │   └── nasa_satellite/
│   │   │       ├── meshes/
│   │   │       │   └── nasa_satellite.dae
│   │   │       ├── model.config
│   │   │       └── model.sdf
│   │   └── ...
│   └── demos/
│       ├── canadarm/
│       │   ├── worlds/
│       │   │   └── simple.world
│       │   └── ...
│       ├── canadarm_moveit_config/
│       └── ...
```

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

실행 후 생성되는 디렉토리 구조:
```
~/space_ros_ws/src/
├── actuator_msgs/
├── demos/
├── demo_manual_pkgs.repos
├── gps_msgs/
├── gz_ros2_control/
├── qt_gui_core/
├── ros2_controllers/
├── ros_gz/
├── simulation/
└── vision_msgs/
```

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

빌드 후 디렉토리 구조:
```
~/space_ros_ws/
├── build/
├── install/
├── log/
└── src/
```

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

## 13. Gazebo Garden 카메라 뷰 설정 및 최적화

아래는 Gazebo Garden 환경에서 NASA 위성의 카메라 뷰를 제대로 설정하고 최적화하는 방법입니다.

### 13.1 SDF 파일에 카메라 GUI 플러그인 설정
`~/space_ros_ws/src/demos/canadarm/worlds/simple.world` 파일을 편집하여 다음 내용을 추가합니다:

```xml
<gui fullscreen="0">
  <!-- 3D 뷰 플러그인 -->
  <plugin filename="GzScene3D" name="3D View">
    <ignition-gui>
      <title>3D View</title>
      <property type="bool" key="showTitleBar">true</property>
      <property type="string" key="state">docked</property>
    </ignition-gui>
    <engine>ogre2</engine>
    <scene>scene</scene>
    <ambient_light>0.4 0.4 0.4</ambient_light>
    <background_color>0 0 0 1</background_color>
    <camera_pose>10 -10 10 0 0.6 2.3</camera_pose>
  </plugin>

  <!-- 카메라 뷰 플러그인 -->
  <plugin filename="ImageDisplay" name="Image Display">
    <gz-gui>
      <title>Camera View</title>
      <property type="bool" key="showTitleBar">true</property>
      <property type="string" key="state">docked</property>
    </gz-gui>
    <topic>nasa_satellite/camera</topic>
    <refresh_rate_hz>60</refresh_rate_hz>
  </plugin>
</gui>
```

### 13.2 카메라 센서 설정 최적화
`~/space_ros_ws/src/simulation/models/nasa_satellite/model.sdf` 파일에서 카메라 센서 설정을 다음과 같이 최적화합니다:

```xml
<sensor name="satellite_camera" type="camera">
  <pose>0 1.2 0 0 0 0</pose>
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
  </camera>
  <always_on>true</always_on>
  <alwaysOn>true</alwaysOn>
  <update_rate>15</update_rate>
  <visualize>true</visualize>
  <topic>nasa_satellite/camera</topic>
</sensor>
```

### 13.3 카메라 뷰 업데이트 문제 해결
카메라 뷰가 실시간으로 업데이트되지 않는 문제가 있을 수 있습니다. 이 경우 다음 방법을 시도해 볼 수 있습니다:

1. **GUI에서 수동으로 카메라 뷰 추가**:
   - 상단 메뉴에서 `Windows` > `Add new plugin` 클릭
   - 목록에서 `ImageDisplay` 선택
   - 토픽 이름으로 `nasa_satellite/camera` 입력

2. **카메라 센서 태그에 다양한 형태의 항상 켜짐 옵션 추가**:
   ```xml
   <always_on>true</always_on>
   <alwaysOn>true</alwaysOn>
   ```

3. **카메라 새로고침 속도 최적화**:
   ```xml
   <update_rate>15</update_rate>
   <refresh_rate_hz>60</refresh_rate_hz>
   ```

---

## 14. 모델 분리 및 Include로 사용하기

NASA 위성 모델은 이미 별도의 파일로 존재합니다. 이를 월드 파일에서 include로 불러올 수 있습니다.

### 14.1 NASA 위성 모델 파일 위치
모델 파일은 이미 다음 위치에 존재합니다:
```
~/space_ros_ws/src/simulation/models/nasa_satellite/model.sdf
```

### 14.2 월드 파일에서 모델 Include 사용
`~/space_ros_ws/src/demos/canadarm/worlds/simple.world` 파일을 편집하여 기존 모델 정의 대신 다음과 같이 include 사용:

```xml
<!-- NASA 위성 모델 인클루드 -->
<include>
  <uri>model://nasa_satellite</uri>
  <name>nasa_satellite</name>
  <pose>-2 -10.7 0.3 0 0 0.8708</pose>
</include>
```

### 14.3 환경 변수 확인
모델 경로가 제대로 설정되어 있는지 확인:
```bash
echo $GZ_SIM_RESOURCE_PATH
```
결과에 `~/space_ros_ws/install/simulation/share/simulation/models`가 포함되어 있어야 합니다.

---

## 15. 자주 발생하는 오류 및 해결책

| **오류 메시지**                                      | **원인**                                         | **해결책**                                                            |
|------------------------------------------------------|--------------------------------------------------|-----------------------------------------------------------------------|
| `rosdep init` 이미 초기화됨                           | rosdep이 이미 초기화됨                             | 무시하고 `rosdep update` 실행                                          |
| `colcon mixin add` 오류                                | colcon-mixin 미설치                                | `sudo apt install python3-colcon-mixin -y` 후 재시도                    |
| OpenGL/GLX 오류 ("Failed to create OpenGL context")    | GPU 가속/X 서버 설정 문제                           | `export LIBGL_ALWAYS_SOFTWARE=1`로 설정                                |
| Gazebo 창이 검은 화면에서 바로 꺼짐                  | X서버 설정 불량 또는 OpenGL 오류                    | VcXsrv "Disable access control" 체크, DISPLAY=:0, LIBGL_ALWAYS_SOFTWARE 적용 |
| `ros2 control load_controller` 서비스 오류          | Gazebo 실행 또는 컨트롤러 초기화 타이밍 문제          | Gazebo 및 컨트롤러 매니저 초기화 상태 확인 후 재시도                      |
| Duplicate package "warehouse_ros_mongo" 오류        | apt 패키지와 소스 클론(또는 repos 파일) 중복         | 소스 빌드를 위해 apt 패키지 제거(또는 repos 파일을 통해 한 번만 내려받기)   |
| "Engine [] is not supported" 오류                   | Gazebo Garden의 렌더링 엔진 설정 문제              | SDF 파일에 `<engine>ogre2</engine>` 명시적 설정                      |
| 카메라 뷰가 업데이트되지 않음                        | 카메라 센서 동기화 문제                            | `<update_rate>` 값 높이기, `<always_on>true</always_on>` 확인      |
| GzImageDisplay 플러그인을 찾을 수 없음              | Gazebo Garden 버전의 플러그인 이름 변경            | `ImageDisplay` 플러그인 사용, `<gz-gui>` 태그 사용                |

---

## 16. 설치 프로그램 및 명령어의 역할

### WSL2 및 Ubuntu 22.04
- **역할:** Windows에서 리눅스 환경 제공 (ROS2, Gazebo 등 실행)

### ROS2 Humble
- **역할:** 로봇 소프트웨어 개발 핵심 프레임워크  
- **설치 이유:**  
  - `ros-humble-desktop`: 기본 GUI 도구 및 기능 포함  
  - `colcon`: 다중 ROS 패키지 빌드  
  - `rosdep`: 의존성 자동 설치

### Gazebo Garden
- **역할:** 가상 시뮬레이션 환경 제공 (최신 Garden 버전)  
- **설치 이유:** ROS2와 연동하여 로봇 동작 테스트 및 최신 OGRE2 기반 렌더링 제공  
- **설치 방법:**  
  - **APT 방식:** 공식 패키지 저장소를 통해 설치  
    - 위 명령어를 통해 `gz-garden`, `ros-humble-ros-gz`, `ros-humble-ign-ros2-control`, 등으로 설치  
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

## 17. 마무리 및 추가 자료

모든 설정과 빌드를 완료하면, WSL2 환경에서 ROS2 Humble, Gazebo Garden, 그리고 Canadarm 시뮬레이션을 안정적으로 실행할 수 있습니다.  
이 가이드는 Open Robotics의 Dockerfile에서 수행된 모든 단계를 로컬에서도 동일하게 재현하고, Gazebo Garden의 카메라 뷰 설정까지 최적화하는 내용을 포함하고 있습니다.

### 17.1 Gazebo Garden 환경에서 GPU 렌더링 활성화 (NVIDIA GPU 사용시)
WSL2에서 NVIDIA GPU를 사용하여 Gazebo Garden의 성능을 향상시키려면:

```bash
# 소프트웨어 렌더링 설정 제거
sed -i '/export LIBGL_ALWAYS_SOFTWARE=1/d' ~/.bashrc

# NVIDIA GPU 렌더링 설정 추가
echo "export DISPLAY=:0" >> ~/.bashrc
echo "export LIBGL_ALWAYS_INDIRECT=0" >> ~/.bashrc
echo "export __NV_PRIME_RENDER_OFFLOAD=1" >> ~/.bashrc
echo "export __GLX_VENDOR_LIBRARY_NAME=nvidia" >> ~/.bashrc
echo "export MESA_GL_VERSION_OVERRIDE=4.5" >> ~/.bashrc

# WSL2 재시작 후 적용
wsl --shutdown
```

### 17.2 GPU 렌더링 확인
```bash
glxinfo | grep "OpenGL renderer"
```
NVIDIA GPU 모델이 표시되면 GPU 가속이 작동하는 것입니다.

### 17.3 Gazebo Garden에서 완벽하게 작동하는 카메라 뷰를 위한 최종 SDF 파일 예시
다음은 3D 뷰와 카메라 뷰가 모두 정상적으로 작동하는 SDF 파일의 완성된 예시입니다:

```xml
<?xml version="1.0"?>
<sdf version="1.10">
  <world name="default">

    <gui fullscreen="0">
      <!-- 3D 뷰 플러그인 -->
      <plugin filename="GzScene3D" name="3D View">
        <ignition-gui>
          <title>3D View</title>
          <property type="bool" key="showTitleBar">true</property>
          <property type="string" key="state">docked</property>
        </ignition-gui>
        <engine>ogre2</engine>
        <scene>scene</scene>
        <ambient_light>0.4 0.4 0.4</ambient_light>
        <background_color>0 0 0 1</background_color>
        <camera_pose>10 -10 10 0 0.6 2.3</camera_pose>
      </plugin>

      <!-- 카메라 뷰 플러그인 -->
      <plugin filename="ImageDisplay" name="Image Display">
        <gz-gui>
          <title>Camera View</title>
          <property type="bool" key="showTitleBar">true</property>
          <property type="string" key="state">docked</property>
        </gz-gui>
        <topic>nasa_satellite/camera</topic>
        <refresh_rate_hz>60</refresh_rate_hz>
      </plugin>

      <!-- 엔티티 컨텍스트 메뉴 -->
      <plugin filename="EntityContextMenuPlugin" name="Entity context menu">
        <ignition-gui>
          <anchors target="3D View">
            <line own="right" target="right"/>
            <line own="top" target="top"/>
          </anchors>
          <property key="state" type="string">floating</property>
          <property key="width" type="double">5</property>
          <property key="height" type="double">5</property>
          <property key="showTitleBar" type="bool">false</property>
        </ignition-gui>
      </plugin>

      <!-- 씬 매니저 -->
      <plugin filename="GzSceneManager" name="Scene Manager">
        <ignition-gui>
          <property type="bool" key="showTitleBar">false</property>
        </ignition-gui>
      </plugin>
    </gui>

    <!-- 씬 설정 -->
    <scene>
      <ambient>0.2 0.2 0.2 1.0</ambient>
      <background>0 0 0 1</background>
      <shadows>true</shadows>
      <grid>false</grid>
    </scene>

    <!-- 중력 설정 (우주 환경) -->
    <gravity>0 0 0</gravity>

    <!-- 조명 설정 -->
    <light type="directional" name="sun">
      <pose>0 -10 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>10 10 -0.9</direction>
    </light>

    <!-- 지구 모델 -->
    <model name="earth">
      <pose>170 0 -50 0 0 -1.5708</pose>
      <static>true</static>
      <link name='link'>
        <inertial>
          <mass>0.25</mass>
          <inertia>
            <ixx>1</ixx> <iyy>1</iyy> <izz>1</izz>
          </inertia>
        </inertial>
        <visual name='visual'>
          <geometry>
            <mesh>
              <uri>model://canadarm/meshes/earth.dae</uri>
              <scale>3 3 3</scale>
            </mesh>
          </geometry>
        </visual>
      </link>
    </model>

    <!-- ISS 모델 -->
    <model name="iss">
      <pose>1 -0.7 -2.3 0 0 1.5708</pose>
      <static>true</static>
      <link name="link">
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>1</ixx> <iyy>1</iyy> <izz>1</izz>
          </inertia>
        </inertial>
        <visual name='visual'>
          <geometry>
            <mesh>
              <uri>model://canadarm/meshes/iss.dae</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
        </visual>
        <velocity_decay>
          <linear>0.0</linear>
          <angular>0.0</angular>
        </velocity_decay>
      </link>
    </model>

    <!-- NASA 위성 모델 인클루드 -->
    <include>
      <uri>model://nasa_satellite</uri>
      <n>nasa_satellite</n>
      <pose>-2 -10.7 0.3 0 0 0.8708</pose>
    </include>

    <!-- 시스템 플러그인 -->
    <plugin filename="libgz-sim-sensors-system.so" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="libgz-sim-physics-system.so" name="gz::sim::systems::Physics"></plugin>
    <plugin filename="libgz-sim-user-commands-system.so" name="gz::sim::systems::UserCommands"></plugin>
    <plugin filename="libgz-sim-scene-broadcaster-system.so" name="gz::sim::systems::SceneBroadcaster"></plugin>

  </world>
</sdf>
```

### 17.4 중요 포인트 정리
1. **GUI 플러그인 이름과 태그**: Gazebo Garden 7.9.0에서는 일부 플러그인에서 여전히 이전 스타일의 태그(`ignition-gui`)를 사용해야 하고, 일부는 새로운 태그(`gz-gui`)를 사용해야 합니다.
2. **카메라 업데이트 문제**: 카메라 센서가 실시간으로 업데이트되지 않는 문제는 WSL2 환경에서 종종 발생합니다. 센서 설정 최적화와 약간의 움직임을 통해 해결할 수 있습니다.
3. **성능 최적화**: 소프트웨어 렌더링은 디버깅에 유용하지만, 성능을 위해서는 GPU 렌더링으로 전환하는 것이 좋습니다.
4. 윈도우에 백업하고 나서 zone identifier 파일이 엄청 생기는데 그거 없애는 명령어는 아래와 같다
   ```bash
   find . -name "*:Zone.Identifier" -type f -delete
   ```
### 17.5 추가 자료
- [ROS2 Humble 공식 문서](https://docs.ros.org/en/humble/)
- [Gazebo Garden 공식 문서](https://gazebosim.org/docs/garden)
- [Space ROS 공식 리포지토리](https://github.com/space-ros)
- [Gazebo Garden GUI 설정 가이드](https://gazebosim.org/api/gui/7.0/gui_config.html)
- [VcXsrv 다운로드 및 설정 안내](https://sourceforge.net/projects/vcxsrv/)
- [WSL2에서 GPU 가속 설정](https://docs.microsoft.com/windows/wsl/tutorials/gpu-compute)
