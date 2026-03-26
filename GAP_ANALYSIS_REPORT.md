# spaceVision 실제 궤도 시뮬레이터 갭 분석 보고서

**작성일**: 2026-03-25
**목적**: 현재 시네마틱 렌더러 → 실제 궤도/우주카메라 환경 시뮬레이터로의 발전을 위한 전수 점검

---

## 1. 현재 상태 요약

### 보유 자산
| 영역 | 현황 | 성숙도 |
|------|------|--------|
| 궤도역학 (MATLAB) | CW + J2 모델, MPC 제어, RK4 전파 | ★★★★☆ |
| 좌표 변환 파이프라인 | MATLAB ECI → CSV → UE Scene 좌표 | ★★★☆☆ |
| 지구 비주얼 | 80K VT 텍스처, 대기/구름/오로라 | ★★★★★ |
| 렌더링 파이프라인 | SceneCapture2D × 4카메라, Python 루프 | ★★★☆☆ |
| 카메라 모델 | 이상적 핀홀 (FOV만 설정) | ★☆☆☆☆ |
| 조명/태양 | DirectionalLight 고정 방향, 강도 20 | ★☆☆☆☆ |
| 시간 체계 | 초 단위 상대시간 (절대 시각 없음) | ★☆☆☆☆ |
| 자세 역학 | Yaw 1축 only, 쿼터니언 없음 | ★★☆☆☆ |
| 센서 모델 | AER 가우시안 노이즈 (MATLAB only) | ★★☆☆☆ |

### 완료된 렌더링
- 180프레임 × 4카메라 = 720장 (50스텝 간격 샘플링)
- 출력: imager(5°/2K), wide30(30°/1K), wide80(80°/1K), gray(30°/1K)

---

## 2. 갭 분석 — 10개 핵심 영역

---

### 2.1 시간 체계 (Time System)

**현재**: 시뮬레이션 시작부터 초(seconds) 카운트. 절대 시각(epoch) 없음.

**필요 사항**:

| 항목 | 설명 | 우선도 |
|------|------|--------|
| UTC Epoch 도입 | 시뮬레이션 시작 시각을 UTC로 정의 (예: 2026-06-15T12:00:00Z) | **필수** |
| Julian Date 변환 | UTC ↔ JD ↔ MJD 변환 함수 (태양/지구 자전 계산에 필수) | **필수** |
| GMST/GAST 계산 | Greenwich Mean Sidereal Time — ECI↔ECEF 변환에 필요 | **필수** |
| 시간 동기화 | MATLAB 시뮬 시간 → UE 렌더 시간 일대일 매핑 | **필수** |
| 윤초 처리 | GPS 시간 vs UTC 오프셋 관리 | 선택 |

**왜 중요한가**: 태양 위치, 지구 자전각, 일식 계산, 조명 방향이 전부 절대 시각에 의존한다. 현재 지구 자전(`37.5° / 9000s`)은 임의의 근사값이며, 실제 지구 자전속도(360°/86164s = 0.00417°/s)와 맞지 않는다.

**현재 오류**:
```
현재: 37.5° / 9000s = 0.00417°/s ← 우연히 거의 정확함
실제: 360° / 86164s = 0.00418°/s
차이: ~0.2% (이것은 우연의 일치, 절대 시각 없이는 초기 방향을 알 수 없음)
```

**구현 방안**:
```matlab
% MATLAB 측
epoch_utc = datetime(2026,6,15,12,0,0,'TimeZone','UTC');
JD_epoch = juliandate(epoch_utc);
for i = 1:N_steps
    JD = JD_epoch + (i * dt) / 86400;
    GMST = siderealTime(JD);  % 그리니치 항성시
    % ECI → ECEF 변환에 GMST 사용
    % 태양 벡터 계산에 JD 사용
end
```

---

### 2.2 태양 위치 및 조명 (Solar Illumination)

**현재**: DirectionalLight 고정 방향/강도(20). 시간에 따른 변화 없음. 일식 계산 없음.

**필요 사항**:

| 항목 | 설명 | 우선도 |
|------|------|--------|
| 태양 ECI 벡터 계산 | JD 기반 태양 위치 (저정밀: 평균 근점이상, 고정밀: VSOP87/JPL DE) | **필수** |
| 프레임별 광원 방향 갱신 | 매 타임스텝마다 DirectionalLight 방향 업데이트 | **필수** |
| 일식(Eclipse) 판정 | 위성이 지구 그림자 내인지 원추/원통 모델로 판정 | **필수** |
| 반지구광(Earthshine) | 지구 반사광 — 야간 측에서도 미약한 조명 | 높음 |
| 알베도 조명 | 지구 표면 반사에 의한 위성 조명 (BRDF 기반) | 선택 |
| 태양광 스펙트럼 | AM0 스펙트럼 (1,361 W/m², 5,778K 흑체) | 선택 |

**현재 오류**: 태양이 항상 같은 방향에서 비추므로, 궤도 한 바퀴 도는 동안 조명 변화가 없다. 실제로는 ~90분 궤도 주기 동안 태양-위성 각도가 계속 변하고, ~30분간 일식 구간이 존재한다.

**구현 방안**:
```matlab
% 저정밀 태양 위치 (충분히 정확, 오차 < 1°)
function r_sun_ECI = sun_position_ECI(JD)
    T = (JD - 2451545.0) / 36525;           % J2000 세기
    M = 357.5291 + 35999.0503 * T;          % 평균 근점이상 (°)
    L = 280.4664 + 36000.7698 * T;          % 평균 황경 (°)
    lambda = L + 1.9146*sind(M) + 0.02*sind(2*M);  % 진황경
    epsilon = 23.439 - 0.013*T;              % 황도 경사각
    AU = 149597870700;                       % 1 AU (m)
    r = AU * (1.00014 - 0.01671*cosd(M));
    r_sun_ECI = r * [cosd(lambda);
                     cosd(epsilon)*sind(lambda);
                     sind(epsilon)*sind(lambda)];
end
```

```python
# UE 측 — 매 프레임 DirectionalLight 방향 업데이트
sun_dir = Vector(sun_x - cam_x, sun_y - cam_y, sun_z - cam_z)
sun_dir.normalize()
sun_light.set_actor_rotation(sun_dir.rotation())

# 일식 판정
if is_in_shadow(sat_pos, earth_center, earth_radius, sun_dir):
    sun_light.set_intensity(0.0)  # 완전 일식
else:
    sun_light.set_intensity(20.0)
```

---

### 2.3 카메라 광학 모델 (Camera Optics)

**현재**: 이상적 핀홀 카메라. FOV만 설정. 렌즈 왜곡, 센서 특성, 노이즈 없음.

**필요 사항**:

| 항목 | 설명 | 우선도 |
|------|------|--------|
| 실제 카메라 스펙 정의 | 초점거리, 센서 크기, 픽셀 피치 → FOV 도출 | **필수** |
| 렌즈 왜곡 모델 | 방사 왜곡 (k1, k2, k3) + 접선 왜곡 (p1, p2) — Brown-Conrady 모델 | 높음 |
| PSF / MTF | Point Spread Function, Modulation Transfer Function — 광학 해상도 한계 | 높음 |
| 센서 노이즈 | 읽기 노이즈, 샷 노이즈, 암전류, 고정 패턴 노이즈 | 높음 |
| 동적 범위 | HDR 캡처 (현재 LDR 8bit → 12/14bit RAW 필요) | 높음 |
| 스미어/블루밍 | CCD 센서 특유의 밝은 물체 주변 번짐 | 선택 |
| 바이어 패턴 | RAW 센서 데이터 시뮬레이션 (디모자이킹 전 상태) | 선택 |

**현재 FOV 검증**:
```
현재 imager: FOV 5° → 초점거리 ≈ 23.4mm (1" 센서 기준)
             또는 ≈ 468mm (1/2" 센서 기준)
문제: 센서 크기가 정의되지 않아 FOV만으로는 실제 광학계를 특정할 수 없음
```

**구현 방안 — 포스트 프로세스 기반 왜곡**:
```python
# 방법 1: UE PostProcess Material로 렌즈 왜곡 적용
# 방법 2: 렌더링 후 Python/OpenCV로 왜곡 적용 (더 정확)
import cv2
import numpy as np

# Brown-Conrady 왜곡 계수 (카메라 캘리브레이션에서 획득)
K = np.array([[fx, 0, cx],
              [0, fy, cy],
              [0,  0,  1]])
dist_coeffs = np.array([k1, k2, p1, p2, k3])

# 이상적 렌더 → 왜곡 적용
distorted = cv2.undistort(ideal_render, K, dist_coeffs)  # 역방향
```

**구현 방안 — 센서 노이즈**:
```python
# 렌더 후 노이즈 추가 파이프라인
def add_sensor_noise(image, params):
    # 1. 포톤 샷 노이즈 (포아송)
    photon_count = image * params['full_well'] / 255.0
    noisy = np.random.poisson(photon_count).astype(float)

    # 2. 읽기 노이즈 (가우시안)
    read_noise = np.random.normal(0, params['read_noise_e'], image.shape)
    noisy += read_noise

    # 3. 암전류
    dark = params['dark_current_e_per_s'] * params['exposure_time']
    noisy += np.random.poisson(dark, image.shape)

    # 4. ADC 양자화
    noisy = np.clip(noisy / params['full_well'] * (2**params['bit_depth'] - 1),
                    0, 2**params['bit_depth'] - 1).astype(np.uint16)
    return noisy
```

---

### 2.4 스케일 및 좌표 정밀도 (Scale & Coordinate Precision)

**현재**: 지구 반경 1,082km (실측의 17%). 좌표값 ~1.2억 cm에서 float 정밀도 문제 존재.

**필요 사항**:

| 항목 | 설명 | 우선도 |
|------|------|--------|
| 실측 스케일 전환 | 지구 반경 6,371km로 확대 (MERGE_PLAN Phase 2-4) | **필수** |
| LWC 정밀도 검증 | 6,371km 스케일에서 cm 단위 정밀도 확인 (UE 5.7 LWC = double) | **필수** |
| 원점 리베이싱 | 카메라 근처를 원점으로 재설정하여 정밀도 확보 | 높음 |
| WGS84 편원체 | 지구를 완벽한 구가 아닌 편원체로 모델링 | 선택 |
| 지형 고도 모델 (DEM) | 지표면 고도 변화 반영 | 선택 |

**현재 정밀도 분석**:
```
현재 좌표 최대값: ~1.2 × 10^8 cm
float32 유효 숫자: ~7자리
→ 1.2 × 10^8에서 정밀도: ~10cm (현재도 이미 문제)

실측 스케일 전환 시: ~6.4 × 10^8 cm
→ float32 정밀도: ~64cm (랑데부 시뮬레이션에 치명적)

해결: UE 5.7 LWC는 내부적으로 double precision 사용
→ double 유효 숫자: ~15자리 → 0.001mm 정밀도 확보 가능
→ 단, SceneCapture2D 렌더링 파이프라인이 double을 끝까지 유지하는지 확인 필요
```

**시퀀서 사용 불가 문제**: 이미 확인됨 — LWC 환경에서 시퀀서 키프레임이 float 정밀도 문제로 작동하지 않음. SceneCapture + Python 루프 방식은 올바른 우회 전략이나, 장기적으로 C++ 모듈에서 직접 제어하는 방안 검토 필요.

---

### 2.5 자세 역학 및 표현 (Attitude Dynamics)

**현재**: MATLAB에서 Yaw 1축만 제어. 쿼터니언 없음. UE에서는 `make_rot_from_xz`로 LVLH 프레임 근사.

**필요 사항**:

| 항목 | 설명 | 우선도 |
|------|------|--------|
| 3축 자세 표현 | 쿼터니언 (q = [q0, q1, q2, q3]) 또는 MRP 사용 | **필수** |
| 자세 역학 방정식 | 오일러 방정식: Iω̇ = -ω × (Iω) + τ_ext + τ_ctrl | **필수** |
| 쿼터니언 전파 | q̇ = 0.5 × Ω(ω) × q (RK4로 적분) | **필수** |
| 외란 토크 모델 | 중력 경사 토크, 자기 토크, 태양 복사압 토크, 공기 저항 토크 | 높음 |
| ADCS 시뮬레이션 | 리액션 휠, 자력 토커, 별 추적기, 태양 센서, 자이로 | 높음 |
| 타겟 자세 독립 제어 | Target 위성의 자세도 별도 시뮬레이션 (현재 고정) | 높음 |

**현재 문제**:
- Chaser 카메라의 LVLH 자세가 UE에서 `make_rot_from_xz(fwd, up)`으로 즉시 계산됨 → 자세 역학 없이 이상적 지향
- Target 위성은 위치만 이동하고 자세(회전)가 고정됨 → 비현실적
- MATLAB MPC에 Yaw만 있어 Roll/Pitch 제어 불가

**구현 방안**:
```matlab
% 쿼터니언 기반 자세 전파
function q_dot = attitude_kinematics(q, omega)
    % q = [q0; q1; q2; q3] (스칼라 먼저)
    Omega = [0,       -omega(1), -omega(2), -omega(3);
             omega(1),  0,        omega(3), -omega(2);
             omega(2), -omega(3),  0,        omega(1);
             omega(3),  omega(2), -omega(1),  0       ];
    q_dot = 0.5 * Omega * q;
end

% 오일러 방정식
function omega_dot = attitude_dynamics(omega, I, tau)
    omega_dot = I \ (tau - cross(omega, I * omega));
end
```

```python
# UE 측 — 쿼터니언 → Rotator 변환
from scipy.spatial.transform import Rotation
R = Rotation.from_quat([q1, q2, q3, q0])  # scipy는 스칼라 마지막
euler = R.as_euler('xyz', degrees=True)
actor.set_actor_rotation(unreal.Rotator(euler[1], euler[2], euler[0]))
```

---

### 2.6 궤도역학 고도화 (Orbital Mechanics Fidelity)

**현재**: J2 섭동까지 구현. 기타 섭동력 없음.

**필요 사항**:

| 항목 | 설명 | 우선도 |
|------|------|--------|
| 고차 지구 중력장 | J2 외에 J3, J4, 또는 EGM96/EGM2008 (차수 ≥ 20) | 중간 |
| 대기 저항 | 지수 대기 모델 또는 NRLMSISE-00 | 높음 |
| 태양 복사압 (SRP) | 면적-질량비 기반 가속도 | 중간 |
| 제3체 인력 | 달/태양 인력 (장기 전파 시 중요) | 선택 |
| 상대 궤도 비선형 모델 | CW → Tschauner-Hempel (이심률 포함) 또는 Full Nonlinear | 높음 |

**현재 분석**:
```
궤도 고도: 545km (a_ref - R_e = 6,923,137 - 6,378,137 = 545km)
이 고도에서 섭동 크기 (가속도 비율):
- J2:   ~10^-3 × g   ← 현재 구현됨
- 대기 저항: ~10^-7 × g (545km에서 매우 작지만 장기적 영향)
- J3:   ~10^-6 × g
- SRP:  ~10^-7 × g
- 달/태양: ~10^-7 × g

시뮬레이션 시간 150분 (1.57 궤도): J2만으로 충분
장기 시뮬레이션 (수일 이상): 대기 저항, J3+ 필요
```

**결론**: 현재 150분 시뮬레이션에서는 J2가 지배적이므로 궤도역학 자체는 적절하다. 다만 장기 미션 시뮬레이션으로 확장 시 대기 저항 추가 필요.

---

### 2.7 우주 배경 환경 (Space Background)

**현재**: StarDome(반경 70,000km), GalaxyRoot(숨김 처리). 물리적 정확도 없음.

**필요 사항**:

| 항목 | 설명 | 우선도 |
|------|------|--------|
| 별 카탈로그 기반 배경 | Hipparcos/Tycho-2 카탈로그 → ECI 방향에 맞는 별 배치 | 높음 |
| 별 등급(magnitude) 반영 | 카메라 감도에 따른 가시 등급 제한 | 높음 |
| 지구 림(limb) 모델 | 지구 가장자리 대기 산란 → 얇은 파란 띠 | 중간 |
| 적절한 스카이박스 크기 | StarDome이 모든 천체를 포함하도록 확대 | 중간 |
| 우주 잡광(stray light) | 태양 직사광/반사광에 의한 플레어, 고스트 | 선택 |

**현재 문제**:
- StarDome 반경 70,000km → 달 궤도(65,284km)를 겨우 포함. 실측 전환 시 달이 StarDome 밖으로 나감
- 별 분포가 물리적 카탈로그 기반이 아님 → VBN 알고리즘 테스트에 사용 불가
- GalaxyRoot를 숨겼지만, 실제 은하수는 VBN에서 중요한 배경 참조

---

### 2.8 렌더 타겟 및 출력 포맷 (Render Output)

**현재**: RTF_RGBA8_SRGB (8bit sRGB), SCS_FINAL_COLOR_LDR. PNG 출력.

**필요 사항**:

| 항목 | 설명 | 우선도 |
|------|------|--------|
| HDR 렌더 타겟 | RTF_RGBA16F 또는 RTF_RGBA32F → 실제 센서 동적 범위 시뮬레이션 | **필수** |
| SceneColor 캡처 | SCS_FINAL_COLOR_HDR 또는 SCS_SCENE_COLOR_NO_ALPHA | **필수** |
| EXR 출력 | HDR 데이터 보존을 위한 OpenEXR 포맷 | 높음 |
| 깊이(Depth) 맵 | SceneCapture로 Z-buffer 출력 → 거리 측정 GT | 높음 |
| 세그멘테이션 맵 | Stencil/CustomDepth로 물체별 마스크 출력 | 높음 |
| 옵티컬 플로우 | 연속 프레임 간 픽셀 이동량 (VBN 검증용) | 중간 |

**왜 중요한가**:
- 우주에서 태양-조명 면과 그림자 면의 밝기 비는 10,000:1 이상
- 8bit LDR (256 레벨)로는 이 동적 범위를 표현할 수 없음
- VBN 알고리즘은 실제 센서 출력과 유사한 입력이 필요

**구현 방안**:
```python
# HDR 렌더 타겟 설정
rt = unreal.RenderTarget2D()
rt.init_custom_format(2048, 2048,
    unreal.EPixelFormat.PF_FLOAT_RGBA,  # RGBA16F
    False)  # linear color space

capture.capture_source = unreal.SceneCaptureSource.SCS_FINAL_COLOR_HDR

# EXR 출력 (UE 내장 지원)
unreal.RenderingLibrary.export_render_target(None, rt, path, name)
# → .hdr 파일 생성 가능, 또는 Python에서 직접 EXR 저장
```

---

### 2.9 데이터 파이프라인 자동화 (Pipeline Automation)

**현재**: MATLAB에서 수동 실행 → CSV 수동 생성 → UE Python 수동 실행.

**필요 사항**:

| 항목 | 설명 | 우선도 |
|------|------|--------|
| MATLAB → CSV 자동 변환 | 시뮬레이션 완료 시 자동 CSV 생성 + 태양 벡터/자세 포함 | **필수** |
| CSV 스키마 확장 | 현재 위치만 → 자세(쿼터니언), 태양 벡터, 일식 플래그, 시각(JD) 추가 | **필수** |
| UE 배치 렌더러 | 커맨드라인에서 UE 에디터 로드 → Python 실행 → 종료 자동화 | 높음 |
| 메타데이터 출력 | 각 프레임의 GT(Ground Truth) 데이터 (위치, 자세, 상대거리 등) JSON 동시 출력 | 높음 |
| 파라미터 스윕 | 다양한 궤도/카메라/조명 조건을 자동 순회하며 데이터셋 생성 | 중간 |

**확장된 CSV 스키마 제안**:
```
Step, Time_s, JD,
Chaser_X, Chaser_Y, Chaser_Z,
Chaser_VX, Chaser_VY, Chaser_VZ,
Chaser_Q0, Chaser_Q1, Chaser_Q2, Chaser_Q3,
Target_X, Target_Y, Target_Z,
Target_VX, Target_VY, Target_VZ,
Target_Q0, Target_Q1, Target_Q2, Target_Q3,
Sun_X, Sun_Y, Sun_Z,
Eclipse_Flag,
RelDist_m, RelVel_ms
```

---

### 2.10 VBN 알고리즘 통합 (Vision-Based Navigation)

**현재**: 렌더링만 수행. VBN 알고리즘 연동 없음.

**필요 사항**:

| 항목 | 설명 | 우선도 |
|------|------|--------|
| Ground Truth 라벨링 | 각 프레임에 대해 타겟 위치/자세/바운딩박스 GT 자동 생성 | **필수** |
| 특징점 가시성 | 큐브샛의 어떤 면/엣지가 카메라에 보이는지 계산 | 높음 |
| 렌더-네비게이션 루프 | 렌더 이미지 → VBN 처리 → 상태 추정 → 다음 제어 입력 (closed-loop) | 높음 |
| 합성 데이터셋 생성 | 다양한 조건(거리, 조명, 자세)의 학습용 데이터셋 자동 생성 | 중간 |
| 성능 메트릭 | VBN 추정 vs GT 비교 (위치 오차, 자세 오차, 수렴 속도) | 중간 |

**왜 중요한가**: 이 시뮬레이터의 궁극적 목적이 VBN 알고리즘 개발/검증이라면, 렌더링 자체가 목적이 아니라 VBN의 입력을 생성하는 도구여야 한다. 현재는 "예쁜 영상"은 만들지만 "쓸 수 있는 데이터"는 만들지 못하는 상태.

---

## 3. 우선순위 로드맵

### Phase A — 물리적 기반 확립 (기본 인프라)

```
[A1] 시간 체계 도입 (UTC epoch, JD, GMST)
[A2] 태양 위치 계산 (ECI 태양 벡터)
[A3] CSV 스키마 확장 (시각, 태양, 자세 포함)
[A4] HDR 렌더 타겟 전환 (RGBA16F + HDR 캡처)
[A5] 프레임별 조명 방향 갱신
```

**예상 작업**: MATLAB 함수 3~4개 추가, CSV 변환 스크립트 수정, UE Python 렌더 루프 수정
**의존성**: A1 → A2 → A3 → A5, A4는 독립

### Phase B — 카메라 및 센서 현실화

```
[B1] 실제 카메라 스펙 정의 (초점거리, 센서 크기, 픽셀 피치)
[B2] 일식 판정 및 조명 on/off
[B3] 3축 자세 도입 (쿼터니언, MATLAB + UE)
[B4] 렌즈 왜곡 모델 (포스트프로세스)
[B5] 센서 노이즈 파이프라인 (Python 후처리)
[B6] Depth/Segmentation 맵 출력
```

**의존성**: A 완료 후 시작. B3은 MATLAB 측 대규모 수정 필요.

### Phase C — 실측 스케일 전환

```
[C1] MERGE_PLAN Phase 1 실행 (BP_Earth80K 내부 분석)
[C2] 스케일 전환 설계 및 테스트
[C3] 에셋 마이그레이션
[C4] 실측 스케일 적용 (지구 6,371km)
[C5] LWC 정밀도 검증
```

**의존성**: 독립적으로 진행 가능하나, B3 이후에 통합하는 것이 효율적

### Phase D — VBN 통합 및 데이터셋

```
[D1] Ground Truth 자동 생성 파이프라인
[D2] 별 카탈로그 기반 배경
[D3] 파라미터 스윕 자동화
[D4] VBN 알고리즘 closed-loop 연동
[D5] 성능 메트릭 대시보드
```

---

## 4. 영역별 상세 갭 매트릭스

| # | 항목 | 현재 상태 | 목표 상태 | 갭 크기 | 우선도 |
|---|------|----------|----------|---------|--------|
| 1 | 시간 체계 | 상대 초 | UTC/JD/GMST | **대** | 필수 |
| 2 | 태양 위치 | 고정 방향 | JD 기반 ECI 벡터 | **대** | 필수 |
| 3 | 일식 모델 | 없음 | 원추/원통 그림자 | **대** | 필수 |
| 4 | 카메라 스펙 | FOV만 | 초점거리/센서/픽셀 정의 | **중** | 필수 |
| 5 | 렌더 포맷 | 8bit LDR | 16bit+ HDR | **중** | 필수 |
| 6 | CSV 스키마 | 위치만 | 위치+속도+자세+태양+시각 | **중** | 필수 |
| 7 | 3축 자세 | Yaw only | 쿼터니언 6DOF | **대** | 필수 |
| 8 | 렌즈 왜곡 | 없음 (이상 핀홀) | Brown-Conrady k1~k3,p1,p2 | **중** | 높음 |
| 9 | 센서 노이즈 | 없음 | 샷/리드/암전류/FPN | **중** | 높음 |
| 10 | 반지구광 | 없음 | 알베도 기반 간접광 | **소** | 높음 |
| 11 | Depth 맵 | 없음 | Z-buffer 출력 | **소** | 높음 |
| 12 | Segmentation | 없음 | Stencil 마스크 | **소** | 높음 |
| 13 | GT 라벨링 | 없음 | 프레임별 JSON 메타데이터 | **중** | 높음 |
| 14 | 별 카탈로그 | 임의 StarDome | Hipparcos/Tycho-2 | **대** | 높음 |
| 15 | 대기 저항 | 없음 | NRLMSISE-00 | **중** | 중간 |
| 16 | 타겟 자세 | 고정 | 독립 자세 역학 | **중** | 높음 |
| 17 | 스케일 | 17% 축소 | 실측 6,371km | **대** | 높음 |
| 18 | 지구 자전 | 근사 (37.5°/9000s) | GMST 기반 정확 자전 | **소** | 필수 |
| 19 | 배치 자동화 | 수동 실행 | CLI 자동화 | **중** | 중간 |
| 20 | Closed-loop VBN | 없음 | 렌더→VBN→제어 루프 | **대** | 중간 |
| 21 | WGS84 편원체 | 구형 | oblate spheroid | **소** | 선택 |
| 22 | SRP/3체 인력 | 없음 | 태양 복사압 + 달/태양 인력 | **소** | 선택 |
| 23 | 옵티컬 플로우 | 없음 | 프레임 간 모션 벡터 | **중** | 선택 |
| 24 | 노이즈 스펙트럼 | 없음 | AM0 스펙트럼 고려 | **소** | 선택 |

---

## 5. 즉시 실행 가능한 Quick Win 목록

아래 항목들은 기존 코드에 소규모 수정만으로 큰 현실성 향상을 달성할 수 있다:

### QW-1. HDR 렌더 타겟 전환 (작업량: 소)
```python
# 현재
rt.init_custom_format(2048, 2048, PF_B8G8R8A8, True)
capture_source = SCS_FINAL_COLOR_LDR

# 변경
rt.init_custom_format(2048, 2048, PF_FloatRGBA, False)
capture_source = SCS_FINAL_COLOR_HDR
```

### QW-2. Depth 맵 동시 출력 (작업량: 소)
```python
# 별도 SceneCapture2D 추가 (DepthCapture)
depth_cap.capture_source = SCS_SCENE_DEPTH
```

### QW-3. 태양 벡터 CSV 추가 (작업량: 소)
```matlab
% cubesat_MPC_cw_v2.m 수정 — 저정밀 태양 위치 함수 추가
% CSV에 Sun_X, Sun_Y, Sun_Z 컬럼 추가
```

### QW-4. 프레임별 메타데이터 JSON 출력 (작업량: 소)
```python
import json
meta = {
    "step": st,
    "chaser_pos": [cx, cy, cz],
    "target_pos": [tx, ty, tz],
    "rel_dist_m": float(r["RelDist_m"]),
    "camera_rot": [rot.pitch, rot.yaw, rot.roll],
    "earth_yaw": ey
}
with open(f"{ob}/meta/frame_{s}.json", "w") as f:
    json.dump(meta, f)
```

### QW-5. 일식 판정 추가 (작업량: 소)
```python
# 원통 그림자 모델 (가장 단순)
def is_eclipsed(sat_pos, earth_center, earth_radius, sun_dir):
    to_sat = sat_pos - earth_center
    proj = to_sat.dot(sun_dir)  # 태양 방향 투영
    if proj > 0:  # 태양 쪽에 있으면 일식 아님
        return False
    perp = (to_sat - sun_dir * proj).length()
    return perp < earth_radius
```

---

## 6. 참조 시스템 벤치마크

유사 프로젝트와의 비교:

| 기능 | spaceVision (현재) | PANGU (ESA) | SISPO (TU Delft) | Basilisk (CU Boulder) |
|------|-------------------|-------------|-------------------|----------------------|
| 궤도역학 | J2 + MPC ✅ | 사전계산 | SPICE 기반 | 고충실도 ✅ |
| 카메라 모델 | 핀홀 ⚠️ | 상세 광학 ✅ | 핀홀+왜곡 ✅ | 핀홀+왜곡 ✅ |
| 태양/조명 | 고정 ❌ | 물리 기반 ✅ | 물리 기반 ✅ | 해석적 ✅ |
| 센서 노이즈 | 없음 ❌ | 상세 ✅ | 기본 ✅ | 상세 ✅ |
| 별 배경 | 데코 ❌ | 카탈로그 ✅ | 카탈로그 ✅ | 카탈로그 ✅ |
| 렌더링 품질 | 80K 지구 ✅✅ | 절차적 ⚠️ | Blender ✅ | 없음 ❌ |
| 3축 자세 | 1축 ❌ | N/A | 쿼터니언 ✅ | 쿼터니언 ✅ |
| VBN 연동 | 없음 ❌ | 직접 ✅ | 직접 ✅ | 간접 ⚠️ |
| 실시간성 | 오프라인 | 오프라인 | 오프라인 | 실시간 ✅ |

**spaceVision의 차별적 강점**: 80K 지구 텍스처 + UE5.7 렌더링 품질은 벤치마크 대비 압도적. 이 비주얼 품질을 유지하면서 물리적 정확도를 올리는 것이 핵심 전략.

---

## 7. 결론

### 가장 시급한 3가지
1. **시간 체계 + 태양 위치** — 이것 없이는 조명이 비현실적이고, 모든 후속 작업의 기반
2. **HDR 렌더 + 확장 CSV** — 현재 8bit LDR은 우주 환경의 극단적 동적 범위를 표현 불가
3. **3축 자세 (쿼터니언)** — 1축 Yaw만으로는 실제 위성 거동을 재현할 수 없음

### 현재 시뮬레이터의 위치
"**시각적으로 인상적인 시네마틱 렌더러**"에서 "**물리적으로 신뢰할 수 있는 VBN 시뮬레이터**"로 전환하기 위한 갭은 존재하지만, 기반 인프라(궤도역학 MATLAB, UE 렌더링 파이프라인, SceneCapture 자동화)는 잘 갖추어져 있다. Phase A의 5개 항목을 완료하면 시뮬레이터의 성격이 근본적으로 전환된다.
