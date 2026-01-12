# NOS3 on WSL2 Ubuntu 구축 완벽 가이드

> **환경**: Windows 10 (Build 19044+) + WSL2 + Ubuntu 24.04 신규 설치 기준

---

## 핵심 권장사항

| 항목 | 권장 |
|------|------|
| **Docker** | WSL2 내부에 네이티브 설치 (Docker Desktop 비권장) |
| **Ubuntu** | **22.04 권장** (24.04는 호환성 문제 다수) |
| **파일 위치** | `/home/user/` 사용 (`/mnt/c/` 절대 금지 - 20배 느림) |
| **X11/GUI** | **WSLg 자동 지원** (VcXsrv 불필요) |

---

## 1. WSL2 Ubuntu 설치

### 1.1 PowerShell (관리자)에서 실행

```powershell
# Ubuntu 22.04 권장
wsl --install -d Ubuntu-22.04

# 또는 Ubuntu 24.04 (추가 설정 필요)
wsl --install -d Ubuntu-24.04
```

### 1.2 WSL 초기화 (재설치 시)

기존 배포판 삭제 후 새로 설치:

```powershell
# 1. Ubuntu 터미널 종료 (열려있다면)
wsl --terminate Ubuntu-24.04

# 2. 배포판 삭제 (모든 데이터 삭제됨!)
wsl --unregister Ubuntu-24.04

# 3. 다시 설치
wsl --install -d Ubuntu-24.04
```

> ⚠️ `--unregister`는 해당 배포판의 **모든 파일이 삭제**됨. 백업 필수.

### 1.3 설치된 배포판 확인

```powershell
wsl -l -v
```

출력 예시:
```
  NAME            STATE           VERSION
* Ubuntu-24.04    Running         2
```

### 1.4 WSL 버전 확인

```powershell
wsl --version
```

출력 예시:
```
WSL 버전: 2.6.3.0
커널 버전: 6.6.87.2-1
WSLg 버전: 1.0.71
...
Windows 버전: 10.0.19045.6456
```

> WSLg 버전이 표시되면 GUI 자동 지원됨.

---

## 2. WSL2 설정

### 2.1 Windows 측: `%UserProfile%\.wslconfig`

메모장으로 파일 생성:
```powershell
notepad $env:USERPROFILE\.wslconfig
```

내용:
```ini
[wsl2]
memory=16GB
processors=8
swap=8GB
localhostForwarding=true
nestedVirtualization=true

[experimental]
autoMemoryReclaim=gradual
sparseVhd=true
```

> NOS3는 다중 컨테이너 실행으로 **최소 16GB RAM** 권장

### 2.2 Linux 측: `/etc/wsl.conf`

```bash
sudo nano /etc/wsl.conf
```

내용:
```ini
[boot]
systemd=true

[automount]
enabled=true
options="metadata,uid=1000,gid=1000,umask=022"

[network]
generateHosts=true
generateResolvConf=true
```

### 2.3 설정 적용

```powershell
wsl --shutdown
```

다시 Ubuntu 실행.

---

## 3. 기본 패키지 설치

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y \
    build-essential \
    cmake \
    git \
    curl \
    wget \
    ca-certificates \
    gnupg \
    lsb-release \
    gnome-terminal \
    dbus-x11 \
    x11-xserver-utils
```

> `gnome-terminal`, `dbus-x11`, `x11-xserver-utils`는 NOS3 `make launch`에 필수

---

## 4. Docker 설치

### 4.1 Docker 저장소 추가

```bash
# GPG 키 추가
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg

# 저장소 추가
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
```

### 4.2 Docker 설치

```bash
sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

### 4.3 사용자 권한 설정

```bash
sudo usermod -aG docker $USER
```

**터미널 종료 후 재접속** (또는 `newgrp docker`)

### 4.4 Docker 자동 시작 설정

```bash
sudo systemctl enable docker.service
sudo systemctl enable containerd.service
```

### 4.5 설치 확인

```bash
docker --version
docker run hello-world
```

---

## 5. X11/GUI (WSLg)

### 5.1 WSLg란?

Windows 10 Build 19044+ 에서 자동 지원:
- **VcXsrv, X410 등 외부 X서버 불필요**
- `DISPLAY=:0` 자동 설정
- GPU 가속 지원 (OpenGL via DirectX 12)

### 5.2 WSLg 작동 원리

```
┌─────────────────────────────────────────────────────────┐
│ Windows 10/11                                           │
│  ┌─────────────────────────────────────────────────┐   │
│  │ WSL2 VM                                          │   │
│  │  ┌─────────────────────────────────────────┐    │   │
│  │  │ 시스템 배포판 (숨김)                      │    │   │
│  │  │  • Weston (Wayland 컴포지터)             │    │   │
│  │  │  • XWayland (X11 호환 레이어)            │    │   │
│  │  │  • PulseAudio (오디오)                   │    │   │
│  │  └─────────────────────────────────────────┘    │   │
│  │         ↑ 소켓 공유 (/mnt/wslg/)                │   │
│  │  ┌─────────────────────────────────────────┐    │   │
│  │  │ 사용자 배포판 (Ubuntu)                   │    │   │
│  │  │  • DISPLAY=:0 (자동 설정)                │    │   │
│  │  │  • xeyes, 42 시뮬레이터 등               │    │   │
│  │  └─────────────────────────────────────────┘    │   │
│  └─────────────────────────────────────────────────┘   │
│         ↓ RDP + 공유 메모리 (VAIL)                      │
│  ┌─────────────────────────────────────────────────┐   │
│  │ Windows 데스크톱                                 │   │
│  │  • Linux 앱이 네이티브 창으로 표시               │   │
│  │  • 작업 표시줄 통합, Alt-Tab 지원               │   │
│  └─────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────┘
```

### 5.3 GUI 테스트

```bash
sudo apt install -y x11-apps mesa-utils
xeyes
```

창이 뜨면 정상.

### 5.4 WSLg 문제 해결

| 증상 | 해결책 |
|------|--------|
| `cannot open display` | `wsl --shutdown` 후 재시작 |
| WSLg 버전 미표시 | PowerShell에서 `wsl --update` |
| 검은 화면/깜빡임 | GPU 드라이버 업데이트 |

---

## 6. Ubuntu 24.04 전용 설정

> Ubuntu 22.04 사용 시 이 섹션 건너뛰기

### 6.1 GCC 11 설치 (권장)

NOS3 일부 컴포넌트가 GCC 14에서 컴파일 오류 발생:

```bash
sudo apt install -y gcc-11 g++-11
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 100
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-11 100
```

확인:
```bash
gcc --version  # 11.x.x 출력되어야 함
```

### 6.2 pip 설치

```bash
sudo apt install -y python3-pip
```

### 6.3 Igniter GUI 설치 (선택)

NOS3 GUI 도구 사용 시:

```bash
pip install PySide6 xmltodict --break-system-packages
```

실행:
```bash
make igniter
```

> Igniter 없이 CLI로 `make launch` 사용 가능

---

## 7. NOS3 빌드 및 실행

### 7.1 클론

```bash
cd ~
git clone https://github.com/nasa/nos3.git
cd nos3
```

### 7.2 서브모듈 초기화

```bash
git submodule update --init --recursive
```

> 시간이 걸릴 수 있음 (인터넷 속도에 따라 수분~수십분)

### 7.3 환경 준비

```bash
make prep
```

이 명령어가 수행하는 작업:
- Docker 이미지 다운로드
- 필요한 컨테이너 설정
- 의존성 준비

> 첫 실행 시 수십분~수시간 소요 (인터넷 속도/PC 성능에 따라 다름)

### 7.4 빌드

```bash
make
```

### 7.5 실행

```bash
make launch
```

실행되는 컴포넌트:
- 42 시뮬레이터 (궤도 역학)
- cFS 비행 소프트웨어
- COSMOS/OpenC3 지상국
- NOS Engine 서버
- 각종 하드웨어 시뮬레이터

### 7.6 실행 화면 설명

`make launch` 실행 시 여러 창이 열림:

#### 42 시뮬레이터 창 (3개)

| 창 | 설명 |
|------|------|
| **42 Cam** | 우주선 3D 시각화. 자세(attitude), 회전, 태양 대비 방향 표시. 우주선이 정상이면 천천히 회전 |
| **42 Map** | 지구 위 우주선 위치 (Ground Track). 궤도 경로와 현재 위치 표시 |
| **Unit Sphere Viewer** | 우주선 자세를 단위 구(unit sphere) 위에 시각화. L1, L2, L3 축(LVLH 좌표계)이 표시되어 우주선 방향/회전 상태 확인 |

> **42**: NASA GSFC에서 개발한 오픈소스 궤도/자세 역학 시뮬레이터
> 
> **LVLH** (Local Vertical Local Horizontal): 지구 중심 기준 좌표계. L1=속도 방향, L2=궤도면 수직, L3=지구 방향

#### COSMOS/OpenC3 (웹 브라우저)

지상국 소프트웨어. 브라우저에서 `http://localhost:2900` 접속:

| 기능 | 설명 |
|------|------|
| **Command Sender** | 우주선에 명령 전송 |
| **Telemetry Viewer** | 우주선 텔레메트리 실시간 모니터링 |
| **Packet Viewer** | 수신된 패킷 상세 보기 |
| **Script Runner** | 자동화 스크립트 실행 |
| **Data Extractor** | 텔레메트리 데이터 추출/분석 |

> 텔레메트리 활성화: Command Sender → CFS_RADIO → TO_ENABLE_OUTPUT 전송

#### 터미널 창들

| 터미널 | 내용 |
|--------|------|
| **NOS3 Flight Software** | cFS 비행 소프트웨어 로그. 앱 초기화, 이벤트 메시지 출력 |
| **Simulators** | 하드웨어 시뮬레이터 탭들 (GPS, IMU, CSS, FSS, 등) |
| **NOS Time Driver** | 시뮬레이션 시간 동기화 |

#### 시뮬레이터 종류

| 시뮬레이터 | 시뮬레이션 대상 |
|------------|----------------|
| **sample_sim** | 샘플 센서 |
| **generic_css_sim** | Coarse Sun Sensor (태양 센서) |
| **generic_fss_sim** | Fine Sun Sensor |
| **generic_imu_sim** | Inertial Measurement Unit (관성 측정 장치) |
| **generic_mag_sim** | Magnetometer (자력계) |
| **generic_torquer_sim** | Magnetic Torquer (자기 토커) |
| **generic_reaction_wheel_sim** | Reaction Wheel (반작용 휠) |
| **generic_eps_sim** | Electrical Power System (전력 시스템) |
| **generic_radio_sim** | Radio (통신) |

> 각 시뮬레이터는 42에서 환경 데이터를 받아 해당 하드웨어의 출력을 시뮬레이션

#### 정상 상태 확인

1. **42 창**: 우주선이 보이고 천천히 회전
2. **COSMOS**: 텔레메트리 데이터 수신 중 (활성화 후)
3. **Flight Software 터미널**: `CFE_ES` 초기화 완료 메시지
4. **시뮬레이터 터미널**: `Construction complete` 메시지

### 7.7 중지

```bash
make stop
```

### 7.8 클린 빌드 (문제 발생 시)

```bash
make clean
make
```

### 7.9 주요 Makefile 타겟

| 명령어 | 설명 |
|--------|------|
| `make prep` | Docker 환경 준비 |
| `make` | 전체 빌드 |
| `make launch` | 시뮬레이션 실행 |
| `make stop` | 시뮬레이션 중지 |
| `make clean` | 빌드 파일 삭제 |
| `make debug` | 디버그 모드 실행 |

### 7.10 Ubuntu 24.04 빌드 시 주의사항

Ubuntu 22.04에서도 일부 문제가 보고됨 (Issue #255):
- 42 창이 검은 화면으로 표시
- 일부 인터페이스 연결 실패

**문제 발생 시 시도할 것**:
1. `wsl --shutdown` 후 재시작
2. Docker 컨테이너 재빌드: `make clean && make prep && make`
3. GCC 11 사용 확인: `gcc --version`

> NOS3는 공식적으로 Ubuntu 20.04/22.04 지원. 24.04는 비공식.

---

## 8. 알려진 버그 및 해결책

### 8.1 공유 메모리 (shm) 오류

**증상**: Windows CMD/PowerShell에서 Docker 실행 시
```
Error: mkdir /dev/shm: file exists
```

**해결**: **항상 WSL2 터미널에서 Docker 명령 실행**

**docker-compose.yml 설정**:
```yaml
services:
  simulator:
    shm_size: "2gb"
    ipc: host
```

### 8.2 gnome-terminal 문제 (Issue #513)

**gnome-terminal이란?**
GNOME 데스크톱 환경의 기본 터미널 에뮬레이터. NOS3는 시뮬레이션 실행 시 여러 터미널 창을 열어 각 컴포넌트(42, cFS, COSMOS 등)의 출력을 표시하는데, 이 창들을 gnome-terminal로 생성함.

**증상**: `make launch` 시 다음 오류 반복:
```
Command 'gnome-terminal' not found
Command 'xhost' not found
Error response from daemon: No such container: cosmos-openc3-operator-1
```

**원인**: NOS3 스크립트가 gnome-terminal을 사용하지만 WSL2 Ubuntu에는 미설치. NOS3 팀이 대안 검토 중 (Issue #513).

**해결**: gnome-terminal 및 관련 패키지 설치:

```bash
sudo apt install -y gnome-terminal dbus-x11 x11-xserver-utils
```

**설치 후**:
```bash
make stop      # 기존 컨테이너 정리
make launch    # 다시 실행
```

#### WSL2에서 gnome-terminal 알려진 버그

| 버그 | 증상 | 해결책 |
|------|------|--------|
| **dbus 연결 실패** | `Error creating terminal: Failed to register` | `dbus-x11` 설치 필수, `wsl --shutdown` 후 재시작 |
| **창 프리즈** | 터미널 창이 입력을 받지 않고 멈춤 | `wsl --shutdown` 후 재시작 |
| **Windows 명령 실패** | `ERROR: UtilConnectToInteropServer: connect failed` | gnome-terminal에서 Windows 명령(`.exe`) 실행 불가. WSL 터미널에서 실행 |
| **WSLg 충돌** | GUI 앱이 일정 시간 후 프리즈 | WSLg 업데이트: `wsl --update` |
| **systemd 필요** | gnome-terminal 실행 안됨 | `/etc/wsl.conf`에 `systemd=true` 확인 |

#### gnome-terminal 시작 안될 때

```bash
# 1. dbus 서비스 확인
sudo service dbus status

# 2. dbus 재시작
sudo service dbus restart

# 3. 그래도 안되면 WSL 재시작
wsl --shutdown   # PowerShell에서 실행
```

> ⚠️ gnome-terminal은 무거움 (~300MB 의존성). NOS3 공식 지원 시 xterm 등 경량 대안으로 변경될 수 있음.

### 8.3 42 시뮬레이터 GUI 문제

| 증상 | 해결책 |
|------|--------|
| `No X11 DISPLAY variable` | `echo $DISPLAY` 확인 (`:0`이어야 함), WSL 재시작 |
| 검은 화면 | GPU 드라이버 업데이트, `wsl --shutdown` |
| 컨테이너 크래시 (headless) | Makefile에서 `GUIFLAG` 비활성화 |
| 파싱 오류 `Bogus input` | 설정 파일 인코딩 UTF-8, 줄바꿈 LF 확인 |

### 8.4 COSMOS/OpenC3 연결 오류

**Redis 연결 실패**:
```bash
export OPENC3_CLOUD=local
```

### 8.5 네트워크 IP 문제 (Issue #267)

컨테이너 간 통신 문제 시:
```bash
echo "$(hostname -I | awk '{print $1}') cosmos nos_engine_server" | sudo tee -a /etc/hosts
```

---

## 9. USB 패스스루 (HITL용)

### 9.1 Windows에서 (관리자 PowerShell)

```powershell
winget install dorssel.usbipd-win
usbipd list
usbipd bind --busid=2-4
usbipd attach --wsl --busid=2-4
```

### 9.2 WSL2에서

```bash
sudo apt install -y linux-tools-generic hwdata
sudo usermod -aG dialout $USER
```

**터미널 재접속 필요**

> ⚠️ USB/IP는 1-5ms 지연 발생

---

## 10. 문제 해결 체크리스트

| 문제 | 확인 사항 |
|------|-----------|
| Docker 권한 오류 | `groups`로 docker 그룹 확인, 터미널 재접속 |
| `cannot open display` | `wsl --version`으로 WSLg 확인, `wsl --shutdown` |
| `gnome-terminal` not found | `sudo apt install gnome-terminal dbus-x11 x11-xserver-utils` |
| 느린 빌드/실행 | 파일 위치 확인 (`/mnt/c/` → `/home/` 이동) |
| 컨테이너 통신 실패 | `docker network ls`, IP 설정 확인 |
| Python 패키지 오류 | `pip install 패키지명 --break-system-packages` |
| 컴파일 오류 | GCC 버전 확인 (`gcc --version`) |
| systemctl 오류 | `/etc/wsl.conf`에 `systemd=true` 확인 |

---

## 11. 참고 링크

- [NOS3 GitHub](https://github.com/nasa/nos3)
- [NOS3 문서](https://nos3.readthedocs.io/)
- [Docker 공식 설치 가이드](https://docs.docker.com/engine/install/ubuntu/)
- [WSLg GitHub](https://github.com/microsoft/wslg)

---

## 요약 (설치 순서)

```bash
# 1. WSL2 설정 (.wslconfig, wsl.conf) → wsl --shutdown

# 2. 기본 패키지
sudo apt update && sudo apt upgrade -y
sudo apt install -y build-essential cmake git curl wget ca-certificates gnupg lsb-release gnome-terminal dbus-x11 x11-xserver-utils

# 3. Docker 설치
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
sudo usermod -aG docker $USER
sudo systemctl enable docker.service containerd.service
# 터미널 재접속

# 4. GUI 테스트
sudo apt install -y x11-apps
xeyes

# 5. (Ubuntu 24.04만) GCC 11 + pip
sudo apt install -y gcc-11 g++-11 python3-pip
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 100
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-11 100
pip install PySide6 xmltodict --break-system-packages  # Igniter GUI용 (선택)

# 6. NOS3 빌드 및 실행
cd ~
git clone https://github.com/nasa/nos3.git
cd nos3
git submodule update --init --recursive
make prep    # Docker 환경 준비 (첫 실행 시 오래 걸림)
make         # 빌드
make launch  # 실행
# make stop  # 중지
```
