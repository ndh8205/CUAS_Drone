# Jetson Orin Nano 설치 및 시스템 확인 가이드

---

## 사전 준비

### 필요한 하드웨어
- Jetson Orin Nano Developer Kit
- microSD 카드 (64GB 이상 권장)
- USB-C 전원 어댑터 (5V/3A 이상)
- 모니터, 키보드, 마우스
- 이더넷 케이블 또는 WiFi 동글

### 필요한 소프트웨어
- NVIDIA SDK Manager (호스트 PC용)
- Jetson Linux (L4T) 이미지

---

## 초기 설정

### 1. SD 카드 이미지 플래싱

```bash
# Etcher 또는 dd 명령어를 사용하여 이미지 플래싱
# Linux에서 dd 사용 예시
sudo dd if=jetson-orin-nano-jp5.1.2-sd-card-image.img of=/dev/sdX bs=1M status=progress
```

### 2. 첫 부팅 및 초기 설정
- SD 카드 삽입 후 전원 연결
- Ubuntu 20.04 초기 설정 진행
- 사용자 계정 생성
- 네트워크 설정

### 3. 시스템 업데이트

```bash
sudo apt update
sudo apt upgrade -y
sudo apt dist-upgrade -y
sudo reboot
```

---

## 시스템 정보 확인

### 1. 기본 시스템 정보

```bash
# 시스템 정보 확인
uname -a

# Ubuntu 버전 확인
lsb_release -a

# Jetson 모델 확인
cat /etc/nv_tegra_release

# L4T 버전 확인
head -n 1 /etc/nv_tegra_release

# 보드 정보 확인
sudo cat /proc/device-tree/model
```

### 2. 하드웨어 정보

```bash
# CPU 정보
lscpu

# 메모리 정보
free -h
cat /proc/meminfo | grep MemTotal

# 스토리지 정보
df -h
lsblk

# GPU 정보
sudo tegrastats
```

### 3. 온도 및 전력 모니터링

```bash
# 실시간 시스템 상태 모니터링
sudo tegrastats

# 온도 확인
cat /sys/devices/virtual/thermal/thermal_zone*/temp

# 팬 상태 확인 (있는 경우)
cat /sys/devices/pwm-fan/target_pwm
```

---

## JetPack 구성 요소 확인

### 1. JetPack 버전 확인

```bash
# JetPack 버전 확인
apt show nvidia-jetpack

# 설치된 NVIDIA 패키지 목록
dpkg -l | grep nvidia
```

### 2. CUDA 확인

```bash
# CUDA 버전 확인
nvcc --version

# CUDA 경로 확인
echo $CUDA_HOME
ls /usr/local/cuda/

# CUDA 샘플 컴파일 및 테스트
cd /usr/local/cuda/samples/1_Utilities/deviceQuery
sudo make
./deviceQuery
```

### 3. cuDNN 확인

```bash
# cuDNN 버전 확인
dpkg -l | grep cudnn

# cuDNN 헤더 파일에서 버전 확인
cat /usr/include/cudnn_version.h | grep CUDNN_MAJOR -A 2
```

### 4. TensorRT 확인

```bash
# TensorRT 버전 확인
dpkg -l | grep tensorrt

# Python에서 TensorRT 확인
python3 -c "import tensorrt; print(tensorrt.__version__)"
```

### 5. OpenCV 확인

```bash
# OpenCV 버전 확인
pkg-config --modversion opencv4

# Python에서 OpenCV 확인
python3 -c "import cv2; print(cv2.__version__)"
```

---

## 성능 모드 설정

### 1. 전원 모드 확인 및 변경

```bash
# 현재 전원 모드 확인
sudo nvpmodel -q

# 사용 가능한 전원 모드 목록
sudo nvpmodel -p --verbose

# 최대 성능 모드로 변경 (모드 번호는 모델에 따라 다를 수 있음)
sudo nvpmodel -m 0

# Jetson 클럭 최대로 설정
sudo jetson_clocks

# Jetson 클럭 상태 확인
sudo jetson_clocks --show
```

### 2. 성능 벤치마크

```bash
# GPU 벤치마크
cd /usr/local/cuda/samples/1_Utilities/bandwidthTest
sudo make
./bandwidthTest

# AI 벤치마크
cd /usr/src/tensorrt/samples/sampleMNIST
sudo make
../../bin/sample_mnist
```
