# ORB‑SLAM3 & Pangolin 설치 및 ROS2 연동 실행 가이드

이 가이드는 Ubuntu 22.04 환경에서 Pangolin과 ORB‑SLAM3를 GitHub에서 클론하고, 빌드한 후 ROS2 (Humble)와 연동하여 USB 웹캠(모노 입력)을 통해 실시간 SLAM을 구현하는 방법을 단계별로 설명합니다.

> **목표**  
> - Pangolin 설치 (시각화 라이브러리)  
> - ORB‑SLAM3 클론 및 클린 빌드  
> - ROS2 작업공간에 ORB‑SLAM3 ROS2 wrapper 클론 및 빌드  
> - 웹캠 캘리브레이션 및 설정 파일 작성  
> - ROS2를 통해 ORB‑SLAM3 실행 및 Pangolin 뷰어에서 SLAM 결과 확인

---

## 목차

1. [시스템 준비 및 필수 의존성 설치](#1-시스템-준비-및-필수-의존성-설치)
2. [Pangolin 설치](#2-pangolin-설치)
3. [ORB‑SLAM3 클론 및 클린 빌드](#3-orb-slam3-클론-및-클린-빌드)
4. [ROS2 작업공간 설정 및 ROS2 Wrapper 빌드](#4-ros2-작업공간-설정-및-ros2-wrapper-빌드)
5. [웹캠 캘리브레이션 및 설정 파일 작성](#5-웹캠-캘리브레이션-및-설정-파일-작성)
6. [ROS2 연동 ORB‑SLAM3 실행](#6-ros2-연동-orb-slam3-실행)
7. [문제 해결 및 디버깅 팁](#7-문제-해결-및-디버깅-팁)
8. [결론](#8-결론)

---

## 1. 시스템 준비 및 필수 의존성 설치

### 1.1. 시스템 업데이트 및 Universe 저장소 활성화

터미널에서 순서대로 실행하세요:

```bash
sudo apt update && sudo apt upgrade -y
```

```bash
sudo apt install -y software-properties-common && sudo add-apt-repository universe
```

### 1.2. 필수 라이브러리 및 도구 설치

ORB‑SLAM3 및 Pangolin 빌드에 필요한 패키지를 설치합니다:

```bash
sudo apt install -y build-essential cmake git pkg-config libeigen3-dev libopencv-calib3d-dev libopencv-features2d-dev libopencv-imgproc-dev libopencv-highgui-dev libgl1-mesa-dev libglew-dev freeglut3-dev
```

### 1.3. 설치 확인

```bash
pkg-config --modversion opencv4
pkg-config --modversion eigen3
```

---

## 2. Pangolin 설치

Pangolin은 ORB‑SLAM3에서 3D 시각화를 위해 사용됩니다. 공식 GitHub에서 소스를 클론한 후 빌드 및 설치합니다.

### 2.1. 소스 클론 및 빌드

터미널에서 아래 명령어를 실행하세요:

```bash
mkdir -p ~/tools && cd ~/tools
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
git checkout v0.6
mkdir build && cd build
cmake -DCPP11_NO_BOOST=1 ..
make -j$(nproc)
sudo make install
```

> **참고:**  
> - 빌드 중 문제가 있으면 `-j$(nproc)` 대신 `-j2` 또는 `-j1`로 병렬 작업 수를 줄이세요.

### 2.2. 설치 확인

Pangolin 라이브러리가 `/usr/local/lib`에 설치되었는지 확인합니다:

```bash
ls /usr/local/lib | grep pangolin
```

---

## 3. ORB‑SLAM3 클론 및 클린 빌드

ORB‑SLAM3 소스는 GitHub에서 클론하여 `/home/server/tools/ORB_SLAM3/`에 저장합니다.

### 3.1. 소스 클론

```bash
cd ~/tools
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git
```

### 3.2. Vocabulary 파일 확인

`/home/server/tools/ORB_SLAM3/Vocabulary/` 폴더에 `ORBvoc.txt` 파일이 있어야 합니다.  
만약 없다면, 압축 해제:

```bash
cd ~/tools/ORB_SLAM3/Vocabulary
tar -xvf ORBvoc.txt.tar.gz
```

### 3.3. 클린 빌드

터미널에서 아래 명령어를 실행하여 클린 빌드를 진행합니다:

```bash
cd ~/tools/ORB_SLAM3
rm -rf build && mkdir build && cd build && cmake .. && make -j2
```

*메모리 부족 문제가 발생하면 `-j2` 대신 `-j1` 사용 또는 스왑 파일을 활성화하세요.*

---

## 4. ROS2 작업공간 설정 및 ROS2 Wrapper 빌드

이 단계에서는 ROS2(Humble)와 연동하기 위해 ORB‑SLAM3 ROS2 wrapper 패키지를 클론하고 빌드합니다.

### 4.1. ROS2 작업공간 생성

```bash
mkdir -p ~/orbslam3_ws/src && cd ~/orbslam3_ws/src
```

### 4.2. ROS2 Wrapper 클론

예시로 [EndlessLoops/ORB_SLAM3_ROS2](https://github.com/EndlessLoops/ORB_SLAM3_ROS2.git)를 사용합니다:

```bash
git clone https://github.com/EndlessLoops/ORB_SLAM3_ROS2.git orbslam3_ros2
```

> **주의:**  
> - 클론된 패키지의 package.xml 파일에 정의된 패키지 이름이 "orbslam3"로 되어 있을 수 있으므로, 이후 실행 시 `ros2 run orbslam3 mono ...` 형식으로 사용합니다.

### 4.3. ROS2 Wrapper 빌드

```bash
cd ~/orbslam3_ws
colcon build --symlink-install
source ~/orbslam3_ws/install/setup.bash
```

빌드 후, 다음 명령어로 "orbslam3" 패키지가 목록에 있는지 확인합니다:

```bash
ros2 pkg list | grep orbslam3
```

---

## 5. 웹캠 캘리브레이션 및 설정 파일 작성

웹캠 캘리브레이션 값(예: fx, fy, cx, cy 등)과 ORB‑SLAM3의 내부 파라미터를 포함하는 YAML 파일을 작성합니다.  
파일 경로: `/home/server/tools/ORB_SLAM3/Examples/Monocular/MyWebcam.yaml`

```yaml
%YAML:1.0
---
# Camera parameters
Camera.type: "PinHole"
Camera.width: 640
Camera.height: 480
Camera.fps: 30
Camera.fx: 876.8776
Camera.fy: 845.6340
Camera.cx: 386.1438
Camera.cy: 293.5898
Camera.k1: -7.23572093e-03
Camera.k2: 1.16278947e+00
Camera.p1: 4.31554218e-03
Camera.p2: -3.53908448e-03
Camera.k3: -3.98585306e+00

# ORB Extractor parameters
ORBextractor.nFeatures: 1000
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7

# Viewer parameters
Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3
Viewer.ViewpointX: 0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -1.8
Viewer.ViewpointF: 500
```

파일 저장 후, 권한을 확인합니다:

```bash
chmod 644 /home/server/tools/ORB_SLAM3/Examples/Monocular/MyWebcam.yaml
```

---

## 6. ROS2 연동 ORB‑SLAM3 실행

### 6.1. 웹캠 이미지 퍼블리시 확인

v4l2_camera 노드를 통해 웹캠이 `/image_raw` 토픽에 이미지 데이터를 퍼블리시하는지 확인합니다:

```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:="/dev/video0"
```

토픽 목록에서 `/image_raw`가 보이면 정상입니다.

### 6.2. ORB‑SLAM3 모노 노드 실행

실행 명령어는 아래와 같습니다:

```bash
ros2 run orbslam3 mono /home/server/tools/ORB_SLAM3/Vocabulary/ORBvoc.txt /home/server/tools/ORB_SLAM3/Examples/Monocular/MyWebcam.yaml
```

> **토픽 Remapping (필요한 경우)**  
> 만약 ROS2 wrapper가 내부적으로 `/camera/color/image_raw`를 구독하도록 되어 있다면, remapping 옵션을 추가합니다:
>
> ```bash
> ros2 run orbslam3 mono /home/server/tools/ORB_SLAM3/Vocabulary/ORBvoc.txt /home/server/tools/ORB_SLAM3/Examples/Monocular/MyWebcam.yaml --ros-args -r /camera/color/image_raw:=/image_raw
> ```

실행하면 콘솔에 Vocabulary 로딩, 카메라 및 ORB Extractor 파라미터 출력이 나타나고, Pangolin 뷰어 창이 열리며 SLAM 결과(카메라 포즈, 3D 포인트 클라우드 등)가 실시간으로 표시되어야 합니다.

---

## 7. 문제 해결 및 디버깅 팁

### 7.1. "Waiting for images" 문제

- **토픽 구독 확인:**  
  - ROS2 노드가 구독하는 이미지 토픽이 실제 퍼블리시되는 토픽(`/image_raw`)과 일치하는지 확인하세요.
  - 필요한 경우 remapping 옵션을 사용합니다.

- **이미지 인코딩 및 전처리:**  
  - ROS2 wrapper 내부에서 cv_bridge를 사용하여 컬러 이미지를 "bgr8" 형식으로 받고, `cv::cvtColor()`를 통해 그레이스케일로 변환하는지 확인합니다.
  - 디버깅 로그를 추가하여 이미지가 정상적으로 수신되고 있는지 확인합니다.

### 7.2. 초기화 및 카메라 모션

- 모노 SLAM은 초기화를 위해 충분한 parallax(시차)가 필요합니다.  
  카메라를 좌우로 움직이며, 텍스처가 풍부한 환경에서 촬영하세요.

### 7.3. ROS2 노드 및 카메라 드라이버 동기화

- v4l2_camera 노드가 이미지를 퍼블리시하고 있다면, ORB‑SLAM3 노드가 해당 토픽을 올바르게 구독하도록 remapping 및 토픽 이름을 확인하세요.
- `ros2 topic echo /image_raw`로 이미지 데이터가 계속 퍼블리시되고 있는지 확인합니다.

---

## 8. 결론

이 가이드는 Ubuntu 22.04 환경에서 Pangolin과 ORB‑SLAM3를 GitHub에서 클론 및 빌드하고, ROS2(Humble)와 연동하여 USB 웹캠(모노 입력)으로 실시간 SLAM을 구현하는 전체 과정을 상세히 안내합니다.  
주요 단계는 다음과 같습니다:
- 시스템 업데이트 및 의존성 설치  
- Pangolin 설치 (GitHub 클론 및 빌드)  
- ORB‑SLAM3 소스 클론 및 클린 빌드  
- ROS2 작업공간 생성 및 ROS2 wrapper(orbslam3_ros2) 클론/빌드  
- 웹캠 캘리브레이션 파일(MyWebcam.yaml) 작성  
- v4l2_camera 노드를 통한 이미지 퍼블리시 확인  
- ROS2를 통해 ORB‑SLAM3 모노 노드 실행 (필요시 토픽 remapping 적용)  
- 문제 해결 및 디버깅 팁을 통한 최종 확인

이제 위 단계를 순서대로 따라 실행하면, Pangolin 뷰어에서 실시간 SLAM 결과(카메라 트래킹 및 3D 맵)를 확인할 수 있습니다.

Happy SLAMing!
