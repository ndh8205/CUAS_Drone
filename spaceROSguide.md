# Space ROS & Space Robots Demo: Canadarm 시뮬레이션 환경 구축 가이드 (Docker 기반 – 최신 main 브랜치 사용)

이 가이드는 WSL2 (Ubuntu 22.04) 환경에서 최신 Space ROS (main 브랜치 기반)와 Docker를 사용하여 ROS 2 기반 우주 로보틱스 플랫폼을 설치하고, 공식 데모 소스(예: Canadarm, Mars Rover 데모)를 빌드 및 실행하는 방법을 단계별로 안내합니다.

> **주의:**  
> - Space ROS 기본 Docker 이미지는 ROS 2 핵심 패키지와 Space ROS 관련 코어만 포함합니다.  
> - 데모 애플리케이션(예: Canadarm 데모, Mars Rover 데모) 및 시뮬레이션 자산은 별도의 저장소(~/space_ros/src/demos, ~/space_ros/src/simulation)에 포함되며, 전체 워크스페이스 빌드를 통해 함께 설치해야 합니다.  
> - 이 가이드는 Docker 컨테이너 내부에서 데모를 실행하는 방법을 중심으로 설명합니다.

---

## 목차

1. [전제 조건](#1-전제-조건)
2. [WSL2 및 Ubuntu 22.04 설치/재설치](#2-wsl2-및-ubuntu-2204-설치재설치)
3. [ROS 2 Humble 및 필수 패키지 설치](#3-ros-2-humble-및-필수-패키지-설치)
4. [Docker Desktop, Docker 및 Earthly 설치](#4-docker-desktop-docker-및-earthly-설치)
   - 4.1 [Docker Desktop 설치 및 확인](#41-docker-desktop-설치-및-확인)
   - 4.2 [Docker 권한 부여](#42-docker-권한-부여)
   - 4.3 [Earthly CLI 수동 설치](#43-earthly-cli-수동-설치)
5. [Space ROS 기본 Docker 이미지 빌드 (최신 main 브랜치)](#5-space-ros-기본-docker-이미지-빌드-최신-main-브랜치)
6. [데모 소스 추가 및 전체 워크스페이스 빌드](#6-데모-소스-추가-및-전체-워크스페이스-빌드)
7. [Docker 컨테이너 실행 및 Canadarm 데모 구동](#7-docker-컨테이너-실행-및-canadarm-데모-구동)
8. [문제 해결 및 디버깅 방법](#8-문제-해결-및-디버깅-방법)
9. [최종 확인 및 마무리](#9-최종-확인-및-마무리)

---

## 1. 전제 조건

- **WSL2 및 Ubuntu 22.04**: WSL2에 Ubuntu 22.04가 설치되어 있어야 합니다.
- **Docker Desktop**: Docker Desktop이 설치되어 있고, WSL Integration이 활성화되어 있어야 합니다.
- **ROS 2 Humble 및 기본 개발 도구**: ROS 2 Humble, git, cmake 등 필수 도구가 설치되어 있어야 합니다.
- **필수 도구 설치**:  
  - **rosdep**: `sudo apt install python3-rosdep2`  
  - **colcon**: `sudo apt install python3-colcon-common-extensions`
- **공식 Space ROS Dockerfile 및 데모 저장소**에 접근 가능해야 합니다.

---

## 2. WSL2 및 Ubuntu 22.04 설치/재설치

### 2.1 WSL2 설치 (CMD 또는 PowerShell)
```cmd
wsl --install -d Ubuntu-22.04
```
설치 후:
```cmd
wsl -l -v
```

### 2.2 Ubuntu 삭제 및 재설치
문제가 있을 경우:
```cmd
wsl --terminate Ubuntu-22.04
wsl --unregister Ubuntu-22.04
wsl --install -d Ubuntu-22.04
```

### 2.3 WSL2 리소스 관리 (선택사항)
Windows 홈 디렉토리 내 `.wslconfig` 파일에:
```plaintext
[wsl2]
memory=12GB
processors=4
swap=2GB
```

---

## 3. ROS 2 Humble 및 필수 패키지 설치

### 3.1 시스템 업데이트 및 필수 패키지 설치
```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y git build-essential cmake libssl-dev wget python3-pip python3-dev python3-setuptools python3-wheel ninja-build exiftool astyle ccache clang clang-tidy g++ gcc gdb make rsync shellcheck unzip xsltproc zip
```

### 3.2 ROS 2 저장소 추가 및 ROS 2 Humble 설치

1. **로케일 설정:**
   ```bash
   sudo apt install -y locales
   sudo locale-gen en_US en_US.UTF-8
   sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
   export LANG=en_US.UTF-8
   ```
2. **필수 도구 설치 및 ROS 2 apt 저장소 추가:**
   ```bash
   sudo apt install -y software-properties-common curl gnupg lsb-release
   sudo add-apt-repository universe
   sudo mkdir -p /etc/apt/keyrings
   curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   sudo apt update
   ```
3. **ROS 2 Humble Desktop 설치:**
   ```bash
   sudo apt install -y ros-humble-desktop
   ```
4. **환경 적용 및 확인:**
   ```bash
   source /opt/ros/humble/setup.bash
   ros2 --help
   ```
   (자동 소스를 위해 `echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc` 추가 가능)

---

## 4. Docker Desktop, Docker 및 Earthly 설치

### 4.1 Docker Desktop 설치 및 확인
- [Docker Desktop 공식 웹사이트](https://www.docker.com/products/docker-desktop)에서 설치 후, Settings > Resources > WSL Integration에서 Ubuntu 22.04가 활성화되었는지 확인합니다.
- WSL 터미널에서:
  ```bash
  docker run hello-world
  ```

### 4.2 Docker 권한 부여
```bash
sudo usermod -aG docker $USER
```
터미널을 재시작하고 확인합니다.

### 4.3 Earthly CLI 수동 설치
```bash
wget https://github.com/earthly/earthly/releases/latest/download/earthly-linux-amd64 -O earthly
sudo mv earthly /usr/local/bin/earthly
sudo chmod +x /usr/local/bin/earthly
sudo earthly bootstrap --with-autocomplete
earthly --version
```

---

## 5. Space ROS 기본 Docker 이미지 빌드 (최신 main 브랜치 사용)

1. **Space ROS 코어 클론 (main 브랜치):**
   ```bash
   git clone https://github.com/space-ros/space-ros.git ~/space_ros
   ```
2. **Space ROS 빌드:**
   ```bash
   cd ~/space_ros
   ./build.sh
   ```
3. **Docker 이미지 확인:**
   ```bash
   docker images | grep space-ros
   ```
   (예: `osrf/space-ros  main ...`)

---

## 6. 데모 소스 추가 및 전체 워크스페이스 빌드

공식 데모(예: Canadarm 데모)는 데모 저장소와 시뮬레이션 자산 저장소에 포함되어 있습니다.

1. **데모 저장소 및 시뮬레이션 저장소 클론:**  
   ```bash
   cd ~/space_ros/src
   git clone https://github.com/space-ros/demos.git
   git clone https://github.com/space-ros/simulation.git
   ```
   저장소가 클론되면, 데모와 시뮬레이션 관련 패키지가 `~/space_ros/src` 내에 존재하게 됩니다.
2. **전체 워크스페이스 의존성 설치 및 빌드:**  
   ```bash
   cd ~/space_ros
   sudo apt install -y python3-rosdep2 python3-colcon-common-extensions
   sudo rosdep init   # 이미 init 된 경우 건너뛰세요.
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --symlink-install
   ```
   빌드가 완료되면, 데모 패키지들(예: canadarm, canadarm_moveit_config 등)이 설치되어야 합니다.

---

## 7. Docker 컨테이너 실행 및 Canadarm 데모 구동

### 7.1 컨테이너 실행

1. **호스트에서 X11 접근 권한 부여:**
   ```bash
   xhost +local:docker
   ```
2. **컨테이너 실행:**  
   Space ROS 기본 이미지(예: `osrf/space-ros:main`)를 사용하여 컨테이너를 실행하고, 데모 워크스페이스 결과물이 포함된 경우 다음과 같이 마운트합니다.
   예를 들어, 호스트의 빌드 결과물(`~/space_ros/install`)과 (옵션) 데모 워크스페이스 결과물(`~/demos_ws/install`, 만약 별도로 빌드했다면)‑을 컨테이너 내부에 마운트합니다.
   ```bash
   docker run --rm -it \
     --network host \
     -e DISPLAY=$DISPLAY \
     -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
     -v ~/space_ros/install:/opt/spaceros/install:ro \
     osrf/space-ros:main bash
   ```
   ※ 만약 데모 결과물이 별도로 빌드되었다면, 해당 디렉토리도 함께 마운트합니다.
   
### 7.2 컨테이너 내부 환경 설정 및 데모 실행

1. **컨테이너 내부에서 ROS 환경 소스 확인:**
   ```bash
   source /opt/spaceros/install/setup.bash
   ```
   데모 워크스페이스를 별도로 빌드한 경우에는 해당 setup 파일도 소스합니다.
2. **Canadarm 데모 실행:**
   ```bash
   ros2 launch canadarm canadarm.launch.py
   ```
   이 명령어가 실행되면 Canadarm 로봇 모델이 시뮬레이터(예: Ignition Gazebo 또는 Gazebo 클래식)에서 스폰되고, MoveIt 2 및 RViz가 함께 실행되어 로봇 제어 및 시각화를 확인할 수 있습니다.
   
   만약 URDF/Xacro 파일 관련 오류가 발생하면, 데모 소스 내 참조 경로를 확인하거나, 시뮬레이션 자산 저장소(`simulation`)가 제대로 클론 및 빌드되었는지 확인합니다.

---

## 8. 문제 해결 및 디버깅 방법

- **캐시 문제 대처:**  
  Docker 이미지 빌드 시 캐시 문제가 발생하면, 아래와 같이 `--no-cache` 옵션을 사용하여 빌드합니다.
  ```bash
  docker build --no-cache -t osrf/space-ros:main .
  ```
- **의존성 오류:**  
  `rosdep install` 과정에서 누락된 패키지가 발생하면, `--skip-keys` 옵션을 사용하거나 개별적으로 설치합니다.
- **컨테이너 내부 환경 확인:**  
  ```bash
  echo $ROS_PACKAGE_PATH
  ```
  ROS_PACKAGE_PATH에 데모 패키지들이 포함되어 있는지 확인합니다.
- **빌드 도구 설치:**  
  rosdep와 colcon이 설치되어 있지 않다면,  
  ```bash
  sudo apt install python3-rosdep2 python3-colcon-common-extensions
  ```

---

## 9. 최종 확인 및 마무리

1. **WSL2 및 ROS 환경 확인:**
   ```bash
   wsl -l -v
   lsb_release -a
   source /opt/ros/humble/setup.bash
   ```
2. **Docker 이미지 확인:**
   ```bash
   docker images | grep space-ros
   ```
3. **컨테이너 실행 및 환경 변수 확인:**
   ```bash
   docker ps
   docker exec -it <컨테이너_ID> bash
   echo $ROS_PACKAGE_PATH
   ```
4. **Canadarm 데모 실행:**
   ```bash
   ros2 launch canadarm canadarm.launch.py
   ```
   모든 설정이 제대로 되어 있다면, Canadarm 데모가 시뮬레이터와 RViz를 통해 정상적으로 실행됩니다.

---

## 마무리

이 가이드는 최신 main 브랜치의 Space ROS와 공식 데모 소스(데모, simulation)를 Docker 기반 환경에서 구축하고 실행하는 방법을 안내합니다.  
공식 Space ROS Docker 이미지와 데모 소스 저장소를 올바르게 클론 및 빌드한 후, Docker 컨테이너 내부에서 `ros2 launch canadarm canadarm.launch.py` 명령어로 Canadarm 시뮬레이션을 실행할 수 있습니다.  
모든 단계가 순서대로 성공적으로 진행되면, Canadarm 로봇의 시뮬레이션 결과를 GUI(RViz, Gazebo 등)에서 확인할 수 있습니다.

추가 문의나 문제가 발생하면 관련 로그와 함께 질문해 주세요.

---

**참고자료:**  
- Space ROS 공식 GitHub (space-ros/space-ros): [https://github.com/space-ros/space-ros](https://github.com/space-ros/space-ros)  
- Space ROS Demos GitHub (space-ros/demos): [https://github.com/space-ros/demos](https://github.com/space-ros/demos)  
- Space ROS Simulation GitHub (space-ros/simulation): [https://github.com/space-ros/simulation](https://github.com/space-ros/simulation)  
- Docker Desktop WSL2 통합 문서: [https://docs.docker.com/go/wsl2/](https://docs.docker.com/go/wsl2/)
```
