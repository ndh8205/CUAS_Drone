NASA NOS3의 **Native Build(로컬 직접 빌드)** 과정을 Ubuntu 22.04 LTS 기준으로, 오류 가능성을 최대한 줄이는 **"All-in-One" 디테일 가이드**입니다.

작성자님의 전공(항공우주)과 리눅스 숙련도를 고려하여, 단순 복사-붙여넣기식 명령어가 아니라 \*\*"왜 이 명령어를 치는지"\*\*에 대한 기술적 맥락을 함께 적었습니다. 터미널을 열고 순서대로 진행해 주십시오.

-----

### Phase 1. 사전 환경 구축 (Pre-requisites)

Ubuntu 22.04의 패키지 관리자를 최신화하고, 빌드 시스템(CMake, Make, GCC)과 필수 라이브러리를 설치합니다. NOS3는 Boost, Xerces-C, ZeroMQ를 핵심 미들웨어로 사용합니다.

```bash
# 1. 시스템 패키지 리스트 업데이트 및 업그레이드
sudo apt update && sudo apt upgrade -y

# 2. 필수 빌드 도구 설치
# build-essential: gcc, g++, make 등 포함
# cmake: 빌드 설정 도구
# git: 형상 관리
sudo apt install -y build-essential cmake git

# 3. 핵심 의존성 라이브러리 설치
# libboost-all-dev: C++ 부스트 라이브러리 (NOS 엔진 통신용)
# libxerces-c-dev: XML 파싱 (설정 파일 처리용)
# libzmq3-dev: ZeroMQ (프로세스 간 고속 통신용)
sudo apt install -y libboost-all-dev libxerces-c-dev libzmq3-dev

# 4. 스크립트 언어 환경 설치 (COSMOS 및 유틸리티용)
# python3-pip: 파이썬 패키지 관리자
# ruby-full: COSMOS(지상국)가 Ruby 기반임
sudo apt install -y python3 python3-pip ruby-full
```

-----

### Phase 2. 소스 코드 확보 (Source Code)

일반적인 `git clone`을 하면 안 됩니다. NOS3 내부에 `42`(동역학), `cFS`(비행SW) 등이 서브모듈로 연결되어 있어 반드시 재귀적(Recursive)으로 가져와야 합니다.

```bash
# 1. 홈 디렉토리 또는 작업 공간으로 이동
cd ~

# 2. 소스 코드 클론 (시간이 조금 걸릴 수 있음)
git clone --recursive https://github.com/nasa/nos3.git

# 3. 디렉토리 이동
cd nos3
```

> **📌 Check:** 만약 실수로 `--recursive`를 빼고 받았다면, 폴더 안에서 `git submodule update --init --recursive`를 입력하면 복구됩니다.

-----

### Phase 3. 의존성 스크립트 실행 (Dependencies)

NOS3 개발팀이 만들어둔 설치 스크립트를 실행합니다. Ubuntu 22.04에서는 일부 패키지 이름이 바뀌었을 수 있지만, 최신 `main` 브랜치의 스크립트는 대부분 이를 감지합니다.

```bash
# 1. 42 시뮬레이터 의존성 설치 (OpenGL, GLUT 등 그래픽 라이브러리 포함)
# 설치 중간에 [Y/n] 질문이 나오면 Y를 입력하세요.
sudo ./support/install/components/install_42_dependencies.sh

# 2. COSMOS 의존성 설치 (Ruby Gems 등)
# 주의: 이 과정에서 시간이 꽤 소요됩니다.
sudo ./support/install/components/install_cosmos_dependencies.sh
```

> **⚠️ 중요 (Ubuntu 22.04 이슈):**
> `install_cosmos_dependencies.sh` 실행 중 Ruby Gem 설치 에러가 날 수 있습니다. Ubuntu 22.04부터 시스템 Ruby에 `gem install`을 막는 정책이 강화되었기 때문입니다.
>
> **만약 에러가 발생한다면:** 아래 명령어로 수동 설치를 시도하세요.
>
> ```bash
> # COSMOS 실행에 필요한 핵심 Gem 수동 설치
> sudo gem install rake cmake
> # (보통 스크립트가 실패해도 NOS3 실행에는 큰 지장이 없는 경우가 많으니 일단 넘어갑니다)
> ```

-----

### Phase 4. 빌드 설정 및 컴파일 (Build)

이제 CMake를 통해 빌드 환경을 구성(Configure)하고 컴파일(Build)합니다.

```bash
# 1. 빌드 디렉토리 생성 (Out-of-source build 권장)
mkdir build
cd build

# 2. CMake 설정
# CMakeLists.txt를 읽어 Makefile을 생성합니다.
cmake ..

# 3. 컴파일 실행
# -j$(nproc): CPU 코어 개수만큼 병렬로 빌드하여 속도를 높임 (매우 중요)
make -j$(nproc)
```

**성공 기준:** `[100%] Built target ...` 메시지가 뜨고 에러 없이 종료되어야 합니다.

  * 만약 `warning`은 뜨지만 `error`가 없다면 성공입니다.

-----

### Phase 5. 시뮬레이션 실행 (Launch)

빌드가 완료되면 시뮬레이터를 실행합니다.

```bash
# build 디렉토리 안에서 실행해야 합니다.
make launch
```

**정상 실행 시 화면:**

1.  **NOS3 COSMOS Ground Station:** 메뉴가 많은 윈도우 창이 뜸.
2.  **42 Simulator:** 검은 우주 배경에 위성 모델(큐브샛 등)이 떠 있는 3D 그래픽 창이 뜸.
3.  **터미널:** 로그가 계속 올라감 (cFS 부팅 로그 등).

-----

### Phase 6. Ubuntu 22.04 트러블슈팅 (Troubleshooting)

만약 `make` 과정이나 `make launch`에서 실패한다면 아래 두 가지 케이스일 확률이 90%입니다.

#### Case 1. GCC 11 컴파일 에러 (`-Werror` 관련)

Ubuntu 22.04의 GCC 11 컴파일러는 매우 엄격해서, 예전 코드의 경고(Warning)를 에러(Error)로 처리하여 빌드를 멈출 수 있습니다.

  * **해결법:** `CMakeLists.txt` (최상위 또는 서브모듈 내부)에서 `-Werror` 옵션을 찾아 제거하거나, 아래 명령어로 빌드 시 경고 무시 옵션을 줍니다.
    ```bash
    cmake -DCMAKE_CXX_FLAGS="-w" ..
    make -j$(nproc)
    ```

#### Case 2. COSMOS 실행 시 "Library not found"

COSMOS가 실행되다가 꺼진다면 Ruby 관련 라이브러리 경로 문제일 수 있습니다.

  * **해결법:** `.bashrc`에 환경변수를 추가해야 할 수 있습니다. 일단 실행이 안 되면 에러 로그를 캡처해서 저에게 알려주세요.

-----

### 🚀 Next Action

위 과정을 순서대로 진행해 보시고, **Phase 4 (make)** 단계에서 멈추거나 에러가 뜬다면 그 **에러 메시지의 마지막 10줄**을 복사해서 보여주세요. 바로 해결책을 드리겠습니다.

지금 바로 터미널을 열고 **Phase 1**부터 시작하시면 됩니다\!