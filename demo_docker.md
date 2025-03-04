# WSL2 Ubuntu 22.04에서 Space ROS Demos 실행 가이드

이 문서는 WSL2 Ubuntu 22.04 환경에서 Docker를 활용하여 [Space ROS Demos](https://github.com/space-ros/demos) 리포지토리의 데모 (특히 Canadarm 데모)를 실행하는 방법을 안내합니다.

## 목차
1. [개요](#개요)
2. [요구사항](#요구사항)
3. [Docker 기반 실행 방법 (권장)](#docker-기반-실행-방법-권장)
   - [환경 준비](#환경-준비)
   - [Docker 이미지 빌드](#docker-이미지-빌드)
   - [데모 실행](#데모-실행)
4. [수동 설치 대안 (비권장)](#수동-설치-대안-비권장)
5. [트러블슈팅](#트러블슈팅)
6. [참고 자료](#참고-자료)

---

## 1. 개요

Space ROS Demos는 ROS 2 Humble 기반의 우주 로봇 데모 예제들을 제공하며, 
- **Curiosity Mars 로버 데모**와 
- **Canadarm (우주 로봇 팔) 데모**  
등을 포함합니다.

Docker를 사용하면 복잡한 의존성 문제 없이 필요한 환경을 컨테이너로 격리해 손쉽게 실행할 수 있습니다.

---

## 2. 요구사항

- **운영체제**: Windows (WSL2) Ubuntu 22.04
- **Docker**: Docker Desktop 또는 WSL2 내부 Docker Engine 설치  
  - WSL2에서 Docker가 정상 작동하는지 `docker run hello-world`로 확인합니다.
- **GUI 환경**:
  - **Windows 11**: WSLg 사용 시 별도 설정 없이 GUI 앱 실행 가능  
    - 추가로 `xhost +local:docker` 명령으로 권한 부여 권장
  - **Windows 10**: VcXsrv 또는 Xming 설치 후, 다음 명령 실행  
    ```bash
    export DISPLAY=<Windows_Host_IP>:0.0
    xhost +local:docker
    ```
- **메모리/디스크**: Docker 이미지 빌드 및 실행 시 충분한 자원 필요
- (선택사항) **GPU**: GPU 가속을 사용하려면 Windows용 WSL GPU 지원 드라이버가 필요하며, GPU 옵션(`--gpus all`)을 활성화합니다. GPU가 없다면 해당 옵션은 제거합니다.

---

## 3. Docker 기반 실행 방법 (권장)

### 환경 준비

1. **Docker 설치 및 연동 확인**  
   - Docker Desktop을 설치하고, WSL2와의 통합이 활성화되었는지 확인합니다.

2. **X 서버 설정 (GUI용)**  
   - Windows 11 (WSLg): 별도 설정 없이 사용 가능하나, `xhost +local:docker` 실행 권장
   - Windows 10: Xming 또는 VcXsrv 설치 후 다음 명령 실행  
     ```bash
     export DISPLAY=<Windows_Host_IP>:0.0
     xhost +local:docker
     ```

### Docker 이미지 빌드

1. **Space ROS Docker 저장소 클론**
   ```bash
   git clone https://github.com/space-ros/docker.git
   cd docker
   ```

2. **MoveIt2 이미지 빌드**  
   MoveIt2 이미지는 Canadarm 데모 실행에 필수입니다.
   ```bash
   cd moveit2
   ./build.sh
   ```
   - 이 스크립트를 통해 `openrobotics/moveit2:latest` 이미지가 생성됩니다.

3. **Space Robots 데모 이미지 빌드**
   ```bash
   cd ../space_robots
   ./build.sh
   ```
   - 빌드가 완료되면 `openrobotics/space_robots_demo:latest` 이미지가 생성됩니다.

### 데모 실행

1. **컨테이너 실행**  
   `run.sh` 스크립트를 사용하여 컨테이너를 실행합니다.
   ```bash
   cd ~/docker/space_robots
   ./run.sh
   ```
   - 스크립트는 `--network host` 옵션과 DISPLAY 환경변수를 전달해 GUI 앱 (Gazebo, RViz) 실행을 지원합니다.
   - GPU 옵션(`--gpus all`)이 포함되어 있으니, GPU가 없는 경우 해당 옵션을 제거하세요.

2. **컨테이너 내부에서 데모 실행**
   - **Curiosity Mars 로버 데모** 실행:
     ```bash
     ros2 launch mars_rover mars_rover.launch.py
     ```
   - **Canadarm 데모** 실행:
     ```bash
     ros2 launch canadarm canadarm.launch.py
     ```
   - 명령 실행 후, Gazebo 시뮬레이터와 RViz 창이 뜨며 데모가 실행됩니다.

3. **컨테이너 종료 및 재접속**
   - 컨테이너 셸에서 `exit`로 빠져나오거나, 다른 터미널에서 `docker exec -it <container_name> bash`로 접속할 수 있습니다.
   - 컨테이너 중단은 `docker stop <container_name>`으로 수행합니다.

---

## 5. 트러블슈팅

- **DISPLAY 오류**  
  - 오류 메시지: `QXcbConnection: Could not connect to display`
  - 해결: `xhost +local:docker` 실행 및 DISPLAY 환경변수 설정 확인

- **Docker 권한 문제**  
  - 오류 메시지: `permission denied` 등  
  - 해결: 현재 사용자를 `docker` 그룹에 추가 (`sudo usermod -aG docker $USER`)하고, 터미널 재시작

- **이미지 빌드 오류**  
  - MoveIt2 이미지 빌드 실패 시, 스크립트를 다시 실행하거나 Docker Hub 대신 로컬 빌드 확인

- **GPU 관련 오류**  
  - GPU 옵션으로 인한 오류가 발생하면, `run.sh`에서 `--gpus all` 옵션을 제거하거나 GPU 드라이버 및 WSL2 GPU 지원 상태 확인

- **포트/네트워크 충돌**  
  - `--network host` 사용 시 포트 충돌 발생 가능  
  - 필요한 경우 host 네트워크 대신 특정 포트를 노출하는 방식으로 변경

---

## 6. 참고 자료

- [Space ROS Demos GitHub Repository](https://github.com/space-ros/demos)
- [Space ROS Docker GitHub Repository](https://github.com/space-ros/docker)
- [ROS 2 Humble 공식 문서](https://docs.ros.org/en/humble/)
- [ROS Answers](https://answers.ros.org)
