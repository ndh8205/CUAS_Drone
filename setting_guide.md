# Jetson Orin Nano 시스템 확인 가이드

호스트 PC에서 SDK Manager를 통한 설치

1. 호스트 PC 요구사항
```
OS: Ubuntu 20.04 또는 Ubuntu 22.04 (x86_64)
메모리: 최소 8GB RAM (16GB 권장)
저장공간: 최소 40GB 여유 공간
인터넷: 안정적인 인터넷 연결
```
2. SDK Manager 설치
### https://developer.nvidia.com 에서 계정 생성

### 2. SDK Manager 다운로드
### https://developer.nvidia.com/nvidia-sdk-manager 에서 
### sdkmanager_[version]_amd64.deb 파일 다운로드

### 3. SDK Manager 설치
sudo apt update
sudo apt install ./sdkmanager_*_amd64.deb

### 4. 의존성 패키지 설치
sudo apt install -y libgconf-2-4 libcanberra-gtk-module

### 5. Jetson orin nano 연결

### 6. Jetson SDK에서 jetpack 설치 진행

## 시스템 업데이트 및 정보 확인

### 1. 시스템 업데이트

```bash
sudo apt update
sudo apt upgrade -y
```

### 2. 기본 시스템 정보

```bash
# 시스템 정보 확인
uname -a

# Ubuntu 버전 확인
lsb_release -a

# Jetson 모델 확인
cat /etc/nv_tegra_release

# 보드 정보 확인
sudo cat /proc/device-tree/model
```

### 3. 하드웨어 정보

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

### 4. 온도 및 전력 모니터링

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

### 2. OpenCV 확인

```bash
# OpenCV 버전 확인
pkg-config --modversion opencv4

```

---

