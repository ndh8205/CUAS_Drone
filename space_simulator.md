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
```bash
sudo apt install -y gz-garden ros-humble-ros-gz ros-humble-ign-ros2-control ros-humble-joint-state-publisher-gui ros-humble-xacro ros-humble-robot-state-publisher ros-humble-controller-manager
```
*설치 후 `gz sim --version` 명령어로 Gazebo Garden 버전을 확인하세요.*

### 4.2 소스 빌드 방식 (선택 사항)  
APT 패키지로 제공되지 않거나 최신 소스를 사용하고자 한다면, 아래와 같이 OSRF Gazebo Garden 소스를 직접 빌드할 수 있습니다.

1. **OSRF 저장소 추가 (APT 방식과 동일하게)**
   ```bash
   sudo apt install -y wget lsb-release gnupg
   sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/gazebo-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] \
   http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list
   sudo apt update
   ```
2. **소스 다운로드 및 빌드 (예시)**
   ```bash
   cd ~
   git clone https://github.com/gazebosim/gz-sim.git
   cd gz-sim
   # Garden 버전 태그(예: garden)를 체크아웃 (태그명은 공식 문서를 참고)
   git checkout garden
   mkdir build && cd build
   cmake .. -DCMAKE_BUILD_TYPE=Release
   make -j$(nproc)
   sudo make install
   ```
   *이 방법은 APT 설치 대신 최신 소스나 커스터마이징이 필요한 경우에 사용합니다.*

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

## 13. Gazebo Garden 카메라 뷰 설정 및 최적화

아래는 Gazebo Garden 환경에서 NASA 위성의 카메라 뷰를 제대로 설정하고 최적화하는 방법입니다.

### 13.1 SDF 파일에 카메라 GUI 플러그인 설정
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
위성 모델 내 카메라 센서 설정을 다음과 같이 최적화합니다:
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

실제 개발 환경에서는 모델을 분리하여 include로 불러오는 방식이 더 유지보수에 용이합니다. 아래는 NASA 위성 모델을 분리하는 방법입니다.

### 14.1 모델 디렉토리 구조 생성
```bash
mkdir -p ~/space_ros_ws/src/demos/models/nasa_satellite/meshes
```

### 14.2 NASA 위성 모델 파일 생성 (model.sdf)
```bash
cat << 'EOF' > ~/space_ros_ws/src/demos/models/nasa_satellite/model.sdf
<?xml version="1.0"?>
<sdf version="1.10">
  <model name="nasa_satellite">
    <pose>0 0 0 0 0 0</pose>
    <static>false</static>
    <link name="nasa_satellite_link">
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.001</ixx>
          <iyy>0.001</iyy>
          <izz>0.001</izz>
        </inertia>
      </inertial>
      <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://nasa_satellite/meshes/nasa_satellite.dae</uri>
          </mesh>
        </geometry>
      </visual>
      <collision name="collision">
        <geometry>
          <mesh>
            <uri>model://nasa_satellite/meshes/nasa_satellite.dae</uri>
          </mesh>
        </geometry>
      </collision>
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
    </link>
  </model>
</sdf>
EOF
```

### 14.3 모델 설정 파일 생성 (model.config)
```bash
cat << 'EOF' > ~/space_ros_ws/src/demos/models/nasa_satellite/model.config
<?xml version="1.0"?>
<model>
  <name>NASA Satellite</name>
  <version>1.0</version>
  <sdf version="1.10">model.sdf</sdf>
  
  <author>
    <name>Your Name</name>
    <email>your.email@example.com</email>
  </author>
  
  <description>
    NASA satellite model with camera sensor
  </description>
</model>
EOF
```

### 14.4 월드 파일에서 모델 Include 사용
```xml
<!-- NASA 위성 모델 인클루드 -->
<include>
  <uri>model://nasa_satellite</uri>
  <name>nasa_satellite</name>
  <pose>-2 -10.7 0.3 0 0 0.8708</pose>
</include>
```

### 14.5 메시 파일 복사
```bash
# 기존의 nasa_satellite.dae 메시 파일을 새 위치로 복사
cp ~/space_ros_ws/src/demos/simulation/models/nasa_satellite/meshes/nasa_satellite.dae ~/space_ros_ws/src/demos/models/nasa_satellite/meshes/
```

### 14.6 환경 변수 설정
```bash
echo "export GZ_SIM_RESOURCE_PATH=\$GZ_SIM_RESOURCE_PATH:~/space_ros_ws/src/demos/models" >> ~/.bashrc
source ~/.bashrc
```

---

## 15. 자주 발생하는 오류 및 해결책

| **오류 메시지**                                      | **원인**                                         | **해결책**                                                            |
|------------------------------------------------------|--------------------------------------------------|-----------------------------------------------------------------------|
| `rosdep init` 이미 초기화됨                           | rosdep이 이미 초기화됨                             | 무시하고 `rosdep update` 실행                                          |
| `colcon mixin add` 오류                                | colcon-mixin 미설치                                | `sudo apt install python3-colcon-mixin -y` 후 재시도                    |
| OpenGL/GLX 오류 ("Failed to create OpenGL context")    | GPU 가속/X 서버 설정 문제                           | `export LIBGL_ALWAYS_SOFTWARE=1`로 설정                                |
| Gazebo 창이 검은 화면에서 바로 꺼짐                  | X서버 설정 불량 또는 OpenGL 오류                    | VcXsrv "Disable access control" 체크, DISPLAY=:0, LIBGL_ALWAYS_SOFTWARE 적용 |
| `ros2 control load_controller` 서비스 오류          | Gazebo 실행 또는 컨트롤러 초기화 타이밍 문제          | Gazebo 및 컨트롤러 매니저 초
