# 🚀 WSL2에서 ROS 2 Humble + Gazebo 완벽 설치 가이드

이 문서는 **Windows 10/11의 WSL2(Ubuntu 22.04)** 환경에서  
**ROS 2 Humble + Gazebo (Ignition Gazebo)** 를 설치할 때 발생할 수 있는 **모든 문제와 해결책을 초상세히** 담고 있습니다.

---

## ✅ **사전 준비**

### 1. WSL2 설치 및 Ubuntu 22.04 구성

Windows 관리자 권한으로 실행한 PowerShell에서:

```powershell
wsl --install -d Ubuntu-22.04
```

재부팅 후 Ubuntu 사용자명, 암호 설정.

### 2. 시스템 업데이트

```bash
sudo apt update && sudo apt upgrade -y
```

---

## 🐢 **ROS 2 Humble 설치**

다음 명령을 차례대로 실행합니다. **줄바꿈 없이 한 줄씩 실행 권장**:

```bash
sudo apt install software-properties-common curl gnupg lsb-release -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list
```

ROS2 패키지 설치:

```bash
sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions python3-rosdep python3-vcstool -y
```

환경설정 (`bashrc`에 추가):

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 🐢 **rosdep 초기화 (필수)**

```bash
sudo rosdep init
rosdep update
```

이미 초기화되어있다는 오류가 나면 무시하고 `rosdep update`만 실행.

---

## 🏗️ **Gazebo (Ignition) 설치**

ROS2 Humble과 호환되는 Ignition Gazebo 버전(Fortress)을 설치합니다.

```bash
sudo apt install ros-humble-ros-ign-gazebo ros-humble-ign-ros2-control ros-humble-joint-state-publisher-gui -y
```

> ⚠️ `warehouse_ros_mongo`와 같은 일부 패키지는 Humble에서 공식 지원되지 않으므로, 필요시 소스 빌드를 해야 합니다.

---

## 🚧 **필요한 기타 도구 설치**

### git-lfs 설치 (대형 파일 다운로드 필수)

```bash
sudo apt install git git-lfs -y
git lfs install
```

### xacro 및 추가 도구 설치

```bash
sudo apt install ros-humble-xacro ros-humble-robot-state-publisher ros-humble-controller-manager ros-humble-ros2-control -y
```

---

## 🛠️ **MongoDB C++ Driver 설치 중 오류 해결**

빌드 시 Python 스크립트 문법 오류가 발생할 경우 (`umask = os.umask(022)` 오류):

**해결법**:

- 스크립트에서 `022`를 `0o22`로 수정  
(8진수 표기법 문제)

---

## 🎨 **WSL2의 OpenGL / Gazebo 오류 완벽 해결**

WSL2에서 Gazebo를 실행하면 OpenGL Context 관련 에러가 발생하는 경우가 많습니다.

**가장 확실한 해결책**은 소프트웨어 렌더링 모드입니다:

```bash
echo "export LIBGL_ALWAYS_SOFTWARE=1" >> ~/.bashrc
source ~/.bashrc
```

이후 Gazebo가 정상적으로 실행됩니다.

---

## 📺 **X서버 (XLaunch/Xming/VcXsrv) 설정**

X서버 (Xming 또는 VcXsrv)를 설치하여 WSL의 GUI를 Windows에 표시합니다.

1. **VcXsrv 다운로드**:  
   https://sourceforge.net/projects/vcxsrv/

2. **XLaunch 실행 설정**:
   - Multiple Windows  
   - Display Number: 0  
   - Start no client  
   - 추가 옵션에서 `Disable access control` 체크 (중요)

`.bashrc`에 DISPLAY 설정 추가:

```bash
echo "export DISPLAY=$(grep nameserver /etc/resolv.conf | awk '{print $2}'):0" >> ~/.bashrc
source ~/.bashrc
```

테스트 (창이 뜨면 성공):

```bash
sudo apt install x11-apps -y
xeyes
```

- 만약 `:0.0`로 하면 눈이 안 뜨는 경우가 있으므로 `:0`으로 지정합니다.

---

## 🔥 **Gazebo 실행 예시 (테스트)**

```bash
mkdir -p ~/space_ros_ws/src && cd ~/space_ros_ws/src
git clone https://github.com/space-ros/simulation.git
cd ~/space_ros_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

아래 명령으로 Gazebo 실행 (예시):

```bash
ros2 launch canadarm canadarm.launch.py robot_description_file:=$(ros2 pkg prefix simulation)/share/simulation/models/canadarm/urdf/SSRMS_Canadarm2.urdf.xacro
```

Gazebo 창이 뜨고 모델이 나타나면 성공입니다! 🎉

---

## 🚨 **흔한 오류 및 빠른 해결책**

| 오류 메시지                                     | 원인 및 해결책                                        |
|------------------------------------------------|--------------------------------------------------|
| `colcon mixin add` 명령 오류                  | `sudo apt install python3-colcon-mixin -y` 설치 후 재시도 |
| rosdep init 시 이미 존재한다는 오류             | 정상 상황, `rosdep update`만 실행                 |
| `LIBGL` 또는 `GLX` 관련 오류                 | `export LIBGL_ALWAYS_SOFTWARE=1`로 설정          |
| Gazebo 창이 검은색으로 뜨다가 꺼짐             | XLaunch의 OpenGL 옵션을 off하고, `LIBGL_ALWAYS_SOFTWARE=1` 추가 |
| `ros2 control load_controller` 서비스 오류     | Gazebo가 정상적으로 실행되지 않은 경우 발생하므로, Gazebo부터 해결 |

---
