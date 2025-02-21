# ORB‑SLAM3 설치 및 ROS2 연동 실행 가이드

이 가이드는 Ubuntu 22.04에서 ORB‑SLAM3를 설치, 빌드하고 ROS2 (Humble)와 연동하여 USB 웹캠(모노 입력)으로 실시간 SLAM을 구현하는 방법을 설명합니다.  

- 시스템 및 의존성 준비
- ORB‑SLAM3 클린 빌드
- ROS2 작업공간 설정 및 ROS2 wrapper 빌드
- 웹캠 캘리브레이션 및 설정 파일 작성 (YAML)
- ROS2 노드를 통해 ORB‑SLAM3 실행 (모노)
- 문제 해결 및 디버깅 팁

---

## 목차

1. [시스템 준비 및 의존성 설치](#1-시스템-준비-및-의존성-설치)
2. [ORB‑SLAM3 클린 빌드](#2-orb-slam3-클린-빌드)
3. [ROS2 작업공간 설정 및 ROS2 Wrapper 빌드](#3-ros2-작업공간-설정-및-ros2-wrapper-빌드)
4. [웹캠 캘리브레이션 및 설정 파일 작성](#4-웹캠-캘리브레이션-및-설정-파일-작성)
5. [ROS2 연동 ORB‑SLAM3 실행](#5-ros2-연동-orb-slam3-실행)
6. [문제 해결 및 디버깅 팁](#6-문제-해결-및-디버깅-팁)
7. [결론](#7-결론)

---

## 1. 시스템 준비 및 의존성 설치

### 1.1. 시스템 업데이트 및 Universe 저장소 활성화

터미널에서 아래 명령어를 순서대로 실행하세요:

```bash
sudo apt update && sudo apt upgrade -y
```

```bash
sudo apt install -y software-properties-common && sudo add-apt-repository universe
```

### 1.2. 필수 라이브러리 및 도구 설치

ORB‑SLAM3 빌드를 위해 필요한 패키지를 설치합니다:

```bash
sudo apt install -y build-essential cmake git pkg-config libeigen3-dev libopencv-calib3d-dev libopencv-features2d-dev libopencv-imgproc-dev libopencv-highgui-dev libgl1-mesa-dev libglew-dev freeglut3-dev
```

### 1.3. 설치 확인

- OpenCV 버전:
  ```bash
  pkg-config --modversion opencv4
  ```
- Eigen3 버전:
  ```bash
  pkg-config --modversion eigen3
  ```

---

## 2. ORB‑SLAM3 클린 빌드

ORB‑SLAM3의 소스는 `/home/server/tools/ORB_SLAM3/`에 위치합니다.  
Vocabulary 파일은 `/home/server/tools/ORB_SLAM3/Vocabulary/ORBvoc.txt`에 있어야 합니다.

### 2.1. 클린 빌드 (메모리 부족 문제가 있으면 스왑 공간도 설정)

터미널에서 아래 명령어를 실행하세요:

```bash
cd ~/tools/ORB_SLAM3 && rm -rf build && mkdir build && cd build && cmake .. && make -j2
```

*만약 "Killed signal terminated program cc1plus" 오류가 발생하면, 병렬 빌드 옵션을 `-j2`에서 `-j1`로 낮추거나 스왑 파일을 추가하세요:*

```bash
sudo fallocate -l 4G /swapfile && sudo chmod 600 /swapfile && sudo mkswap /swapfile && sudo swapon /swapfile
```

빌드 완료 후 필요에 따라 스왑을 해제합니다:

```bash
sudo swapoff /swapfile
```

---

## 3. ROS2 작업공간 설정 및 ROS2 Wrapper 빌드

이 가이드는 ROS2 (Humble)를 기준으로 합니다.

### 3.1. ROS2 작업공간 생성

```bash
mkdir -p ~/orbslam3_ws/src && cd ~/orbslam3_ws/src
```

### 3.2. ROS2 Wrapper 클론

예시로, [EndlessLoops/ORB_SLAM3_ROS2](https://github.com/EndlessLoops/ORB_SLAM3_ROS2.git) repository를 사용합니다:

```bash
git clone https://github.com/EndlessLoops/ORB_SLAM3_ROS2.git orbslam3_ros2
```

### 3.3. ROS2 Wrapper 빌드

터미널에서 작업공간 루트로 이동 후 colcon 빌드를 수행합니다:

```bash
cd ~/orbslam3_ws && colcon build --symlink-install
```

빌드가 완료되면, 환경 설정:

```bash
source ~/orbslam3_ws/install/setup.bash
```

*설치된 패키지 목록에 "orbslam3"가 보이면 성공입니다:*

```bash
ros2 pkg list | grep orbslam3
```

---

## 4. 웹캠 캘리브레이션 및 설정 파일 작성

예를 들어, 웹캠으로 사용하던 값(아래 값들)을 이용해 `/home/server/tools/ORB_SLAM3/Examples/Monocular/MyWebcam.yaml` 파일을 작성합니다.

```yaml
%YAML:1.0
---
# Camera parameters
Camera.type: "PinHole"
Camera.width: 640          # 웹캠 해상도 (예: 640)
Camera.height: 480         # 웹캠 해상도 (예: 480)
Camera.fps: 30             # 웹캠 FPS (예: 30)
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

*파일 저장 후, 권한을 확인 (읽기 권한 644):*

```bash
chmod 644 /home/server/tools/ORB_SLAM3/Examples/Monocular/MyWebcam.yaml
```

---

## 5. ROS2 연동 ORB‑SLAM3 실행

ORB‑SLAM3 ROS2 wrapper는 "orbslam3" 패키지 내에 실행 파일들이 포함되어 있습니다. 확인된 실행 파일은 다음과 같습니다:

- `mono` – 모노 카메라 노드
- `rgbd`, `stereo`, `stereo-inertial` 등

### 5.1. 웹캠으로 모노 SLAM 실행

v4l2_camera 노드를 통해 웹캠에서 이미지가 퍼블리시되고 있음을 확인한 후, 아래 명령어를 실행합니다:

```bash
ros2 run orbslam3 mono /home/server/tools/ORB_SLAM3/Vocabulary/ORBvoc.txt /home/server/tools/ORB_SLAM3/Examples/Monocular/MyWebcam.yaml
```

> **주의:**  
> - 만약 ROS2 wrapper의 소스 코드에서 이미지 구독 토픽이 `/camera/color/image_raw`로 설정되어 있다면, remapping 옵션을 추가하세요.
>   예:
>   ```bash
>   ros2 run orbslam3 mono /home/server/tools/ORB_SLAM3/Vocabulary/ORBvoc.txt /home/server/tools/ORB_SLAM3/Examples/Monocular/MyWebcam.yaml --ros-args -r /camera/color/image_raw:=/image_raw
>   ```
> - 웹캠 노드(v4l2_camera)가 퍼블리시하는 토픽은 `/image_raw`로 확인되었습니다.
> - ROS2 작업공간이 올바르게 소스되었는지 확인하세요:
>   ```bash
>   source ~/orbslam3_ws/install/setup.bash
>   ```

실행하면 콘솔에 Vocabulary 로딩 및 카메라 파라미터, ORB 추출기 파라미터 등이 출력되고, Pangolin 뷰어 창이 열리며 SLAM 결과(카메라 포즈, 3D 포인트 클라우드 등)가 실시간으로 표시되어야 합니다.

---

## 6. 문제 해결 및 디버깅 팁

### 6.1. "Waiting for images" 상태

- **토픽 구독:**  
  - ROS2 노드가 구독하는 이미지 토픽이 실제 퍼블리시되는 토픽(`/image_raw`)과 일치하는지 확인합니다.  
  - 필요시 remapping 옵션(`--ros-args -r /camera/color/image_raw:=/image_raw`)을 사용합니다.

- **이미지 전처리:**  
  - ROS2 wrapper의 GrabImage 함수에서 cv_bridge를 사용해 "bgr8" 형식으로 받은 후, `cv::cvtColor`를 이용해 그레이스케일로 변환하여 SLAM에 전달하는지 확인합니다.
  - 디버깅 로그를 추가하여 이미지 프레임 크기와 수신 여부를 확인하세요.

### 6.2. 보정 파일 및 설정 파일 오류

- YAML 파일의 헤더 `%YAML:1.0`이 포함되어 있는지 확인하고, 구문 오류가 없는지 검토합니다.
- 파일 권한 및 경로가 올바른지 점검합니다.

### 6.3. 빌드 오류 및 메모리 부족

- 클린 빌드를 수행하고, 빌드 시 병렬 작업 수를 줄이거나 스왑 공간을 늘려 "Killed signal" 오류를 방지합니다.

---

## 7. 결론

이 가이드는 ORB‑SLAM3의 설치, 빌드, ROS2 연동 및 웹캠(모노) 입력을 통한 실시간 SLAM 구현 방법을 단계별로 안내합니다.  
- **시스템 준비**: 필수 패키지 설치 및 업데이트  
- **ORB‑SLAM3 클린 빌드**: 메모리 부족 시 스왑 공간 추가  
- **ROS2 작업공간 설정**: orbslam3_ros2 패키지 클론 및 colcon 빌드  
- **캘리브레이션 파일 작성**: MyWebcam.yaml에 카메라, ORB extractor, Viewer 파라미터 추가  
- **실행**: `ros2 run orbslam3 mono ...` 명령어로 실행 (remapping 옵션 적용 가능)  
- **문제 해결**: 토픽 이름, 인코딩 변환, 디버깅 로그 등을 통해 이미지 수신 문제 해결

이제 위 단계를 순서대로 따라 실행하면, Pangolin 뷰어에서 실시간 SLAM 결과를 확인할 수 있습니다.

Happy SLAMing!
