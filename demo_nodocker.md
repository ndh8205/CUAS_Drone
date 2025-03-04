# 🚀 WSL2 Ubuntu 22.04에서 Space ROS Canadarm 시뮬레이션 구축 가이드

**환경:** Windows 10/11 (WSL2), Ubuntu 22.04  
**목표:** Docker 없이 Space ROS Canadarm 시뮬레이션 구현  

---

## 📌 목차
1. [WSL2 환경 설정](#1-wsl2-환경-설정)
2. [ROS2 Humble 설치](#2-ros2-humble-설치)
3. [필수 패키지 설치](#3-필수-패키지-설치)
4. [Space ROS 패키지 클론](#4-space-ros-패키지-클론)
5. [워크스페이스 빌드 및 설정](#5-워크스페이스-빌드-및-설정)
6. [시뮬레이션 실행 방법](#6-시뮬레이션-실행-방법)
7. [문제 해결 팁](#7-문제-해결-팁)

---

## ✅ 1. WSL2 환경 설정

**① Ubuntu 22.04 설치**

Windows CMD/PowerShell에서:

```powershell
wsl --install -d Ubuntu-22.04
```

**② WSL2 자원 제한 (선택 권장)**

`C:\Users\<사용자명>\.wslconfig` 작성:

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
빠른 초기화를 위한 명령어:
```bash
wsl --terminate Ubuntu-22.04
wsl --unregister Ubuntu-22.04
wsl --install -d Ubuntu-22.04
```
재부팅 후 Ubuntu 사용자명, 암호 설정.
---

## ✅ 2. ROS2 Humble 설치

**① Locale 설정**

```bash
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

**② ROS2 저장소 키 및 저장소 추가 (줄 바꿈 최소화)**

```bash
sudo apt install -y curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

**③ ROS2 Humble 설치**

```bash
sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions python3-rosdep python3-vcstool
```

**④ 추가 ROS 개발 도구 (선택)**

```bash
sudo apt install -y python3-pip python3-colcon-mixin python3-flake8 python3-pytest-cov python3-rosinstall-generator ros-humble-ament-* ros-humble-ros-testing ros-humble-eigen3-cmake-module
```

**⑤ ROS2 환경 설정**

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo rosdep init || true
rosdep update

colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update
```

**⑥ 설치 확인**

```bash
ros2 --help
ros2 topic list
printenv ROS_DISTRO  # humble 출력 확인
```

---

## ✅ 3. 필수 패키지 설치

```bash
sudo apt install -y ros-humble-ros-gz ros-humble-moveit ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-joint-state-publisher ros-humble-xacro ros-humble-ros-ign-bridge libasio-dev git-lfs
```

## 4. Gazebo(Ignition) 및 ROS 관련 패키지 설치

**목적:**  
Gazebo는 로봇 시뮬레이션 환경으로, ROS와의 연동을 위해 필요한 패키지들을 설치합니다.

```bash
sudo apt install ros-humble-ros-gz ros-humble-ign-ros2-control ros-humble-joint-state-publisher-gui ros-humble-xacro ros-humble-robot-state-publisher ros-humble-controller-manager -y
```

그리고 MoveIt의 데이터베이스 백엔드로 사용할 **warehouse_ros_sqlite**도 설치합니다:

```bash
sudo apt install ros-humble-warehouse-ros-sqlite -y
```

---

## 5. 미지원 패키지 수동 빌드 (Space ROS)

**목적:**  
Space ROS 관련 패키지(예: 시뮬레이션 자산, 데모)는 공식 apt 저장소에 없으므로 GitHub에서 소스를 직접 클론하여 빌드합니다.

1. **작업공간 생성 및 소스 클론:**
   ```bash
   mkdir -p ~/space_ros_ws/src && cd ~/space_ros_ws/src
   git lfs install
   git clone https://github.com/space-ros/simulation.git
   git clone https://github.com/space-ros/demos.git
   ```
   - **설명:**  
     - `simulation`: Canadarm URDF, 모델, Gazebo 월드 파일 등이 포함되어 있습니다.
     - `demos`: Canadarm 시뮬레이션 실행을 위한 데모 코드와 launch 파일이 포함되어 있습니다.

2. **의존성 설치 및 빌드:**
   ```bash
   cd ~/space_ros_ws
   rosdep install --from-paths src --ignore-src -r -y --skip-keys warehouse_ros_mongo
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   source install/setup.bash
   ```
   - **설명:**  
     `--skip-keys warehouse_ros_mongo` 옵션은 ROS Humble에서 해당 패키지가 공식 지원되지 않으므로 건너뛰도록 합니다.

3. **환경 변수 추가:**
   ```bash
   echo "source ~/space_ros_ws/install/setup.bash" >> ~/.bashrc
   echo "export IGN_GAZEBO_RESOURCE_PATH=\$IGN_GAZEBO_RESOURCE_PATH:~/space_ros_ws/install/simulation/share/simulation/models" >> ~/.bashrc
   source ~/.bashrc
   ```

---

## 6. X서버(VcXsrv) 설치 및 GUI 설정

**목적:**  
WSL2에서 실행되는 GUI 애플리케이션(예: Gazebo, Rviz)을 Windows에서 볼 수 있도록 X 서버(VcXsrv 등)를 사용합니다.

1. **VcXsrv 다운로드:**  
   [VcXsrv 다운로드](https://sourceforge.net/projects/vcxsrv/)

2. **XLaunch 설정 (중요):**
   - **Multiple windows** 선택  
   - **Display Number:** 0  
   - **Start no client** 선택  
   - **Disable access control** 옵션 체크 (반드시 체크)

3. **WSL2에서 DISPLAY 환경 변수 설정:**
   ```bash
   echo "export DISPLAY=$(grep nameserver /etc/resolv.conf | awk '{print $2}'):0" >> ~/.bashrc
   source ~/.bashrc
   ```
4. **테스트:**
   ```bash
   sudo apt install x11-apps -y
   xeyes
   ```
   - **설명:** xeyes 창이 뜨면 X 서버 연결이 정상입니다.
   - **주의:** `:0`로 설정해야 제대로 동작합니다. `:0.0`은 문제가 발생할 수 있습니다.

---

## 7. OpenGL 문제 해결 및 환경 변수 설정

**문제:**  
WSL2에서는 GPU 하드웨어 가속이 불안정할 수 있어, Gazebo가 OpenGL 관련 오류로 종료되는 경우가 많습니다.

**해결 방법 (권장 - 소프트웨어 렌더링):**

```bash
echo "export LIBGL_ALWAYS_SOFTWARE=1" >> ~/.bashrc
source ~/.bashrc
```

- **설명:**  
  이 설정은 GPU 대신 CPU 기반 소프트웨어 렌더링을 사용하여 OpenGL 오류를 회피합니다.
  단, 성능은 다소 저하될 수 있습니다.

---

## 8. ROS 작업공간 빌드 및 시뮬레이션 실행

1. **Workspace 빌드:**
   ```bash
   cd ~/space_ros_ws
   colcon build --symlink-install --cmake-clean-cache
   source install/setup.bash
   ```
2. **Canadarm 시뮬레이션 실행:**
   ```bash
   ros2 launch canadarm canadarm.launch.py robot_description_file:=$(ros2 pkg prefix simulation)/share/simulation/models/canadarm/urdf/SSRMS_Canadarm2.urdf.xacro
   ```
   - **설명:**  
     이 명령어는 Canadarm 시뮬레이션을 실행하며, 외부 파일로부터 URDF를 불러옵니다.  
     문제가 없으면 Gazebo 창에 로봇 모델이 나타나야 합니다.

---

## 9. 자주 발생하는 오류 및 빠른 해결책

| **오류 메시지** | **원인** | **해결책** |
|----------------|----------|------------|
| `rosdep init` 이미 초기화됨 | rosdep이 이미 초기화됨 | 무시하고 `rosdep update`만 실행 |
| `colcon mixin add` 오류 | colcon-mixin 패키지가 설치되지 않음 | `sudo apt install python3-colcon-mixin -y` 후 재시도 |
| OpenGL/GLX 오류 ("Failed to create OpenGL context") | GPU 가속 문제, X 서버 설정 문제 | `export LIBGL_ALWAYS_SOFTWARE=1`로 설정 (또는 GPU 가속 환경 구성) |
| Gazebo 창이 검은 화면에서 바로 꺼짐 | X서버 설정 불량 또는 OpenGL 오류 | VcXsrv 설정에서 "Disable access control" 체크 확인, DISPLAY=:0 설정 및 LIBGL_ALWAYS_SOFTWARE 적용 |
| `ros2 control load_controller` 서비스 오류 | Gazebo가 정상적으로 실행되지 않아 컨트롤러 로드 실패 | Gazebo 실행 문제를 먼저 해결한 후 재시도 |

---

## 10. 설치하는 프로그램 및 명령어의 역할

### **WSL2 및 Ubuntu 22.04**
- **역할:** Windows에서 리눅스 환경을 제공하여 ROS2 및 Gazebo와 같은 리눅스 전용 소프트웨어를 실행할 수 있게 합니다.

### **ROS2 Humble**
- **역할:** 로봇 소프트웨어 개발을 위한 핵심 프레임워크입니다.
- **설치 이유:**  
  - `ros-humble-desktop`: ROS2의 GUI 도구와 기본 기능을 포함합니다.
  - `colcon`: 여러 ROS 패키지를 동시에 빌드할 수 있도록 합니다.
  - `rosdep`: ROS 패키지의 의존성을 자동으로 설치합니다.

### **Gazebo (Ignition)**
- **역할:** 가상 환경에서 로봇 동작을 시뮬레이션할 수 있게 해줍니다.
- **설치 이유:**  
  ROS2와 연동되어 실제 로봇 동작을 가상으로 테스트할 수 있도록 해줍니다.

### **X서버 (VcXsrv)**
- **역할:** WSL2에서 실행되는 GUI 애플리케이션을 Windows에서 볼 수 있도록 합니다.
- **설치 이유:**  
  Gazebo, Rviz와 같은 GUI 프로그램의 화면을 표시하기 위해 필수입니다.

### **OpenGL 관련 환경 변수**
- **LIBGL_ALWAYS_SOFTWARE:**  
  GPU 가속 대신 소프트웨어 렌더링을 강제하여, WSL2 환경에서 OpenGL 오류를 회피합니다.
- **DISPLAY:**  
  X서버와의 연결을 설정하여 GUI 창이 Windows에 표시되도록 합니다.

---

## 11. 마무리 및 추가 자료

모든 설정과 빌드를 완료하면 WSL2 환경에서 ROS2 Humble과 Gazebo를 안정적으로 실행할 수 있습니다.  
이 가이드는 ROS2와 Gazebo 설치 과정에서 발생할 수 있는 모든 문제와 그 해결책, 그리고 각 명령어의 목적까지 상세히 설명하여, 사용자가 직접 이해하고 문제를 해결할 수 있도록 도와줍니다.

**추가 자료:**
- [ROS2 Humble 공식 문서](https://docs.ros.org/en/humble/)
- [Space ROS 공식 리포지토리](https://github.com/space-ros)
- [VcXsrv 다운로드 및 설정 안내](https://sourceforge.net/projects/vcxsrv/)

**행복한 로봇 프로그래밍 되세요! 🚀🌙**

---

이 가이드는 대화 내내 지적받은 모든 사항(설치 이유, 명령어의 의미, 문제 발생 시 해결책 등)을 반영하였으므로, 이제 WSL2에서 ROS2 Humble과 Gazebo 및 Canadarm 시뮬레이션을 성공적으로 구축할 수 있을 것입니다.
