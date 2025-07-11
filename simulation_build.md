# 🚀 WSL2 Ubuntu 24.04에서 인공위성 시뮬레이션 구축 가이드 (Gazebo Harmonic 사용)

**환경:** Windows 10/11 (WSL2), Ubuntu 24.04  
**목표:** Docker 없이 최신 환경(ROS 2 Jazzy, Gazebo Harmonic)에서 인공위성(예, NASA 위성) 모델을 구동하는 시뮬레이션을 구축  
**비고:** 본 가이드는 로봇 제어 기능이나 복잡한 컨트롤러 대신, 단순히 인공위성 모델을 Gazebo Harmonic에서 구동시키는 데 필요한 최소한의 패키지와 설정만 포함합니다.  
**참고:** 제어 관련 패키지(ros2_controllers, gz_ros2_control 등)는 필요 없으므로 모두 제외하였으며, 그에 따른 영향(예, 관절 제어 기능 없음 등)은 단순 시각적 시뮬레이션에서는 무관함을 전제로 합니다.

---

## 📌 목차
1. [WSL2 환경 설정](#1-wsl2-환경-설정)
2. [ROS 2 Jazzy 설치](#2-ros2-jazzy-설치)
3. [필수 패키지 설치](#3-필수-패키지-설치)
4. [Gazebo Harmonic 및 ROS 연동 패키지 설치](#4-gazebo-harmonic-및-ros-연동-패키지-설치)
   - 4.1. **APT 설치 방식**
5. [워크스페이스 소스 클론 및 빌드 준비](#5-워크스페이스-소스-클론-및-빌드-준비)
   - 5.1. 시뮬레이션 소스 클론 (simulation, demos)
6. [데모 의존성 소스 코드 가져오기 (통신 모듈 포함 & repos 파일 자동 생성)](#6-데모-의존성-소스-코드-가져오기-통신-모듈-포함--repos-파일-자동-생성)
7. [의존성 설치 및 전체 빌드 (warehouse_ros_mongo 건너뛰기)](#7-의존성-설치-및-전체-빌드-warehouse_ros_mongo-건너뛰기)
8. [환경 변수 및 추가 설정 적용](#8-환경-변수-및-추가-설정-적용)
9. [X서버(VcXsrv) 및 GUI 설정](#9-x서버vcxsrv-및-gui-설정)
10. [OpenGL 문제 해결 (소프트웨어 렌더링)](#10-opengl-문제-해결-소프트웨어-렌더링)
11. [ROS 작업공간 실행 및 인공위성 시뮬레이션 실행](#11-ros-작업공간-실행-및-인공위성-시뮬레이션-실행)
12. [GPU 사용 시 사용자 그룹 추가](#12-gpu-사용-시-사용자-그룹-추가)
13. [Gazebo Harmonic 카메라 뷰 설정 및 최적화](#13-gazebo-harmonic-카메라-뷰-설정-및-최적화)
14. [모델 분리 및 Include로 사용하기](#14-모델-분리-및-include로-사용하기)
15. [빌드 오류 해결 및 종속성 관리](#15-빌드-오류-해결-및-종속성-관리)
16. [설치 프로그램 및 명령어의 역할](#16-설치-프로그램-및-명령어의-역할)
17. [마무리 및 추가 자료](#17-마무리-및-추가-자료)

---

## 1. WSL2 환경 설정

### 1.1 Ubuntu 24.04 설치  
Windows CMD 또는 PowerShell에서 다음 명령어로 Ubuntu 24.04를 설치합니다:
```powershell
wsl --install -d Ubuntu-24.04
```

### 1.2 WSL2 자원 제한 (선택 권장)  
`C:\Users\<사용자명>\.wslconfig` 파일에 다음 내용을 작성하여 메모리, 프로세서, 스왑 영역을 제한합니다:
```ini
[wsl2]
memory=8GB
processors=4
swap=2GB
```
변경 후에는:
```cmd
wsl --shutdown
```
또는 초기화를 위해:
```bash
wsl --terminate Ubuntu-24.04
wsl --unregister Ubuntu-24.04
wsl --install -d Ubuntu-24.04
```
재부팅 후 Ubuntu 사용자명 및 암호를 설정합니다.

---

## 2. ROS 2 Jazzy 설치

> **참고:** ROS 2 Jazzy는 Gazebo Harmonic과 원활하게 연동할 수 있도록 최신 기능이 반영된 ROS 2 배포판입니다.

### 2.1 Locale 설정
```bash
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### 2.2 ROS 2 저장소 키 및 저장소 추가
```bash
sudo apt install -y curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
*Ubuntu 24.04의 코드네임은 시스템에 따라 자동 반영됩니다.*

### 2.3 ROS 2 Jazzy 설치
```bash
sudo apt update
sudo apt install -y ros-jazzy-desktop python3-colcon-common-extensions python3-rosdep python3-vcstool
```

### 2.4 추가 ROS 개발 도구 (선택)
```bash
sudo apt install -y python3-pip python3-colcon-mixin python3-flake8 python3-pytest-cov python3-rosinstall-generator ros-jazzy-ament-* ros-jazzy-ros-testing ros-jazzy-eigen3-cmake-module
```

### 2.5 ROS 2 환경 설정
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
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
printenv ROS_DISTRO  # "jazzy" 출력 확인
```

---

## 3. 필수 패키지 설치
```bash
sudo apt install -y ros-jazzy-ros-gz ros-jazzy-moveit ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-joint-state-publisher ros-jazzy-xacro libasio-dev git-lfs
```
> **중요:** 본 가이드에서는 제어 관련 패키지(ros2_controllers, gz_ros2_control, actuator_msgs 등)는 필요 없으므로 추후 데모 및 빌드에서 사용하지 않습니다.

---

## 4. Gazebo Harmonic 및 ROS 연동 패키지 설치

### 4.1 APT 설치 방식  
공식 OSRF 패키지 저장소를 통해 Gazebo Harmonic과 ROS 연동 패키지들을 설치합니다.

**OSRF 저장소 추가**
```bash
sudo apt install -y wget lsb-release gnupg
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/gazebo-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] \
http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list
sudo apt update
```

**Gazebo Harmonic 및 ROS 연동 패키지 설치**
```bash
sudo apt install -y gz-harmonic ros-jazzy-ros-gz ros-jazzy-joint-state-publisher-gui ros-jazzy-xacro ros-jazzy-robot-state-publisher ros-jazzy-controller-manager
```
*설치 후 `gz sim --version` 명령어로 Gazebo Harmonic 버전을 확인하세요.*

> **주의:** 이 가이드에서는 제어 관련 기능은 사용하지 않으므로, ros‑gz 브리지만을 이용하여 Gazebo와 ROS 간 기본 통신만 수행합니다.

---

## 5. 워크스페이스 소스 클론 및 빌드 준비

### 5.1 시뮬레이션 소스 클론 (simulation, demos)
```bash
mkdir -p ~/space_ros_ws/src && cd ~/space_ros_ws/src
git lfs install
git clone https://github.com/space-ros/simulation.git
git clone https://github.com/space-ros/demos.git
```
*설명:*  
- **simulation:** NASA 위성 모델, 기타 인공위성 관련 모델 파일이 포함되어 있습니다.  
- **demos:** (필요한 경우) 인공위성 시뮬레이션에 참고할 수 있는 예제 world 파일 등이 포함되어 있습니다.

디렉토리 구조:
```
~/space_ros_ws/
├── src/
│   ├── simulation/
│   │   ├── models/
│   │   │   └── nasa_satellite/
│   │   │       ├── meshes/
│   │   │       │   └── nasa_satellite.dae
│   │   │       ├── model.config
│   │   │       └── model.sdf
│   │   └── ...
│   └── demos/
│       ├── satellite/         # (원하는 경우 인공위성 관련 데모가 있을 수 있음)
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
     qt_gui_core:
       type: git
       url: https://github.com/ros-visualization/qt_gui_core.git
       version: jazzy
     ros_gz:
       type: git
       url: https://github.com/gazebosim/ros_gz.git
       version: jazzy
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
이 과정에서 인공위성 시뮬레이션에 필요한 통신 모듈 및 기타 의존 소스 코드만 내려받습니다.

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
- 제어 관련 패키지는 포함하지 않으므로, 인공위성 모델 및 관련 데모만 빌드됩니다.  
- 빌드 오류가 발생하면 기존 빌드 아티팩트(`build/`, `install/`, `log/`)를 삭제하고 클린 빌드를 진행하세요.

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
> **참고:** Ubuntu 24.04와 최신 WSLg 환경에서는 기본 GUI 지원이 있으나, VcXsrv를 사용할 수도 있습니다.

---

## 10. src 파일 추가 및 교체

함께 첨부된 src파일 내부의 파일들을 src로 복사하여 아래의 명령어 수행

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

아래의 코드는 윈도우에서 우분투로 파일을 복사하여 넣었을 때 생기는 Zone.Identifier를 삭제하는 명령어로 
명령어를 사용하여 저 파일들을 모두 지우는 것이 좋습니다.

```bash
find . -name "*:Zone.Identifier*" -delete
```

---

## 11. ROS 작업공간 실행 및 인공위성 시뮬레이션 실행

첨부된 예시의 인공위성 편대비행의 인공위성 모델을 구동하는 시뮬레이션을 실행합니다.

```bash
ros2 launch canadarm sat2.launch.py
```

*설명:* 위 명령어로 Gazebo Harmonic이 실행되면, 인공위성 모델이 정상적으로 구동되어 시각적으로 확인할 수 있습니다.

실제 아루코마커의 포즈 추정 시뮬레이션을 실행합니다.

```bash
ros2 launch aruco_gazebo_controller aruco_simulator.launch.py
```

*설명:* 위 명령어로 Gazebo Harmonic이 실행되면, 인공위성 모델이 정상적으로 구동되어 시각적으로 확인할 수 있습니다.

---


---

## 12. GPU 사용 시 사용자 그룹 추가

```bash
sudo usermod -aG render $USER
```
*참고:* 재부팅 후 적용

---

## 13. Gazebo Harmonic 카메라 뷰 설정 및 최적화

만약 인공위성에 달린 카메라 센서를 활용하여 영상 뷰를 확인하고자 한다면, 아래와 같이 world 파일에 GUI 플러그인 설정을 추가할 수 있습니다.

### 13.1 SDF 파일에 카메라 GUI 플러그인 설정
예를 들어, `~/space_ros_ws/src/demos/satellite/worlds/simple.world` 파일에 다음 내용을 추가하세요:
```xml
<gui fullscreen="0">
  <!-- 3D 뷰 플러그인 -->
  <plugin filename="GzScene3D" name="3D View">
    <gz-gui>
      <title>3D View</title>
      <property type="bool" key="showTitleBar">true</property>
      <property type="string" key="state">docked</property>
    </gz-gui>
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
`~/space_ros_ws/src/simulation/models/nasa_satellite/model.sdf` 파일 내의 카메라 센서 설정을 아래와 같이 조정합니다:
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
    <!-- 필요 시 16비트 깊이 이미지 옵션 추가 -->
    <!-- <depth_format>R16_UINT</depth_format> -->
  </camera>
  <always_on>true</always_on>
  <update_rate>15</update_rate>
  <visualize>true</visualize>
  <topic>nasa_satellite/camera</topic>
</sensor>
```

---

## 14. 모델 분리 및 Include로 사용하기

NASA 위성 모델은 별도의 파일로 이미 존재합니다. 이를 월드 파일에서 include 방식으로 불러오려면:

### 14.1 NASA 위성 모델 파일 위치
```
~/space_ros_ws/src/simulation/models/nasa_satellite/model.sdf
```

### 14.2 월드 파일에서 모델 Include 사용
예를 들어, `~/space_ros_ws/src/demos/satellite/worlds/simple.world` 파일에서 다음과 같이 include 태그를 사용합니다:
```xml
<include>
  <uri>model://nasa_satellite</uri>
  <name>nasa_satellite</name>
  <pose>-2 -10.7 0.3 0 0 0.8708</pose>
</include>
```

### 14.3 환경 변수 확인
```bash
echo $GZ_SIM_RESOURCE_PATH
```
출력 결과에 `~/space_ros_ws/install/simulation/share/simulation/models`가 포함되어 있어야 합니다.

---

## 15. 빌드 오류 해결 및 종속성 관리

최신 공식 버전을 모두 사용 시 제어 관련 패키지는 사용하지 않으므로, 발생할 수 있는 빌드 오류는 주로 **헤더 변경**과 **종속성 누락**에 집중됩니다.

- **헤더 변경 문제:**  
  - 만약 `rclcpp/qos_event.hpp` 관련 오류가 발생하면, 해당 패키지가 더 이상 사용되지 않도록 수정하거나, 소스 코드 내 관련 부분을 제거합니다. (본 가이드에서는 제어 기능이 필요 없으므로 해당 부분은 제거됩니다.)

- **ros_ign_gazebo 종속성 문제:**  
  - canadarm, mars_rover 패키지에서 `ros_ign_gazebo` 키를 찾지 못하는 오류가 발생하면, 해당 의존성을 package.xml에서 제거하고 대신 최신 **ros_gz** 관련 패키지로 대체합니다.

- **클린 빌드:**  
  - 빌드 오류를 방지하려면, 기존 빌드 디렉토리(예: `build/`, `install/`, `log/`)를 삭제 후 클린 빌드를 진행합니다.
  ```bash
  rm -rf build/ install/ log/
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
  ```

- **종속성 재확인:**  
  - `rosdep install --from-paths src --ignore-src -r -y --skip-keys warehouse_ros_mongo` 명령어로 모든 필요한 시스템 종속성이 설치되었는지 확인합니다.

이와 같이 제어 관련 패키지를 완전히 제외하면, 단순히 인공위성 모델만 구동되므로 제어 관련 API나 헤더 변경 문제는 발생하지 않으며, 빌드 시간이 단축되고 복잡성이 크게 낮아집니다.

---

## 16. 설치 프로그램 및 명령어의 역할

(이하 내용은 이전 버전과 동일합니다.)

### WSL2 및 Ubuntu 24.04
- **역할:** Windows에서 최신 리눅스 환경 제공 (ROS 2, Gazebo Harmonic 등 실행)

### ROS 2 Jazzy
- **역할:** 로봇 소프트웨어 개발의 핵심 프레임워크  
- **설치 이유:**  
  - `ros-jazzy-desktop`: 최신 GUI 도구 및 기능 포함  
  - `colcon`: 다중 ROS 패키지 빌드  
  - `rosdep`: 의존성 자동 설치

### Gazebo Harmonic
- **역할:** 고성능 가상 시뮬레이션 환경 제공 (최신 렌더링, 전역 조명, 향상된 센서 지원)  
- **설치 이유:** ROS 2와 연동하여 보다 사실적인 시뮬레이션을 제공  
- **설치 방법:**  
  - **APT 방식:** OSRF 공식 저장소를 통해 설치 (위 명령어 참조)  
  - **소스 빌드 (선택):** 최신 기능이나 커스터마이징이 필요할 경우 소스 빌드 가능

### X서버 (VcXsrv)
- **역할:** WSL2에서 GUI 애플리케이션 화면 표시  
- **설치 이유:** Gazebo Harmonic, RViz 등 GUI 도구 실행

### OpenGL 관련 환경 변수
- **LIBGL_ALWAYS_SOFTWARE:** 소프트웨어 렌더링 강제 (문제 발생 시 임시 적용)  
- **DISPLAY:** X 서버 연결 설정

### 통신 모듈 및 데모 의존성
- **역할:**  
  - `qt_gui_core`, `ros_gz`, `vision_msgs`, `gps_msgs` 등은 인공위성 시뮬레이션에서 필요한 기본 통신 및 시각화 기능을 제공합니다.  
  - demo_manual_pkgs.repos 파일을 통해 필요한 의존 소스 코드를 한 번에 내려받습니다.
> **중요:** 제어 관련 패키지(ros2_controllers, gz_ros2_control, actuator_msgs 등)는 이번 구성에서 제외됩니다.

---

## 17. 마무리 및 추가 자료

모든 설정과 빌드를 완료하면, WSL2 환경에서 Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic 및 업데이트된 인공위성 시뮬레이션(예: NASA 위성 모델)이 안정적으로 실행됩니다.  
본 가이드는 복잡한 로봇 제어 없이 단순히 인공위성 모델만을 구동하는 데 초점을 맞추었으므로, 기존 Canadarm 제어 관련 기능은 모두 제거되었습니다.

### 17.1 Gazebo Harmonic 환경에서 GPU 렌더링 활성화 (NVIDIA GPU 사용시)
WSL2에서 NVIDIA GPU를 사용하여 Gazebo Harmonic의 성능을 극대화하려면:
```bash
# 소프트웨어 렌더링 설정 제거 (필요시)
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
NVIDIA GPU 모델이 출력되면 GPU 가속이 정상 동작하는 것입니다.

다음은 Gazebo Harmonic 및 ROS 2 Jazzy 환경에 맞게 17.3 "브릿지 열기" 섹션을 수정한 업데이트 버전입니다. 이 버전에서는 예전의 ros‑ign‑bridge 대신 최신 ros‑gz 브리지를 사용하며, 메시지 타입도 Gazebo Harmonic에 맞게 네임스페이스가 변경된 것을 반영합니다.


### 17.3 추가 자료
- [ROS 2 Jazzy 공식 문서](https://docs.ros.org/en/jazzy/)
- [Gazebo Harmonic 공식 문서](https://gazebosim.org/docs/harmonic)
- [Space ROS 공식 리포지토리](https://github.com/space-ros)
- [Gazebo Harmonic GUI 설정 가이드](https://gazebosim.org/api/gui/)
- [VcXsrv 다운로드 및 설정 안내](https://sourceforge.net/projects/vcxsrv/)
- [WSL2에서 GPU 가속 설정](https://learn.microsoft.com/windows/wsl/tutorials/gpu-compute)
