# Jetson Orin Nano 시스템 확인 가이드

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

