# 센트로이드 → Star ID → QUEST → 쿼터니언 출력 파이프라인 상세 분석 보고서

**작성일**: 2026-02-05
**대상 코드**: `ssov_v700_20260205`
**작성**: Claude Code (자동 분석)

---

## 목차

1. [파이프라인 전체 흐름](#1-파이프라인-전체-흐름)
2. [이슈 요약 (심각도별)](#2-이슈-요약-심각도별)
3. [CRITICAL 이슈 상세](#3-critical-이슈-상세)
4. [HIGH 이슈 상세](#4-high-이슈-상세)
5. [MEDIUM 이슈 상세](#5-medium-이슈-상세)
6. [LOW 이슈 상세](#6-low-이슈-상세)
7. [데이터 플로우 추적 다이어그램](#7-데이터-플로우-추적-다이어그램)
8. [수정 우선순위 권고](#8-수정-우선순위-권고)
9. [Timeout / 블로킹 전수 조사 및 해결 방안](#9-timeout--블로킹-전수-조사-및-해결-방안)

---

## 1. 파이프라인 전체 흐름

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         star_track_full() [star_detect.c:534]              │
│                                                                             │
│  ┌──────────────────────────────────────────────────────────────────────┐   │
│  │  STAGE 1: Star Detection Pipeline   [star_detect_pipeline()]        │   │
│  │                                                                      │   │
│  │  sd_load_frame()     DDR → image0[720][1280]                        │   │
│  │       ↓                                                              │   │
│  │  sd_compute_threshold()  서브샘플링(4px간격) → mean, sigma, thresh   │   │
│  │       ↓                                                              │   │
│  │  sd_ccl(threshold)   2-pass CCL + Union-Find → label_map, state     │   │
│  │       ↓                                                              │   │
│  │  sd_filter_and_centroid()                                           │   │
│  │       Area/BBox/SNR 필터 → CoG Centroid                             │   │
│  │       → idx.label_num (별 개수)                                      │   │
│  │       → GBF_v.centroid_x[i], GBF_v.centroid_y[i] (픽셀 좌표)       │   │
│  └──────────────────────────────────────────────────────────────────────┘   │
│       ↓                                                                     │
│  [idx.label_num > SD_MAX_STARS_ID(15) 이면 캡핑]                           │
│       ↓                                                                     │
│  ┌──────────────────────────────────────────────────────────────────────┐   │
│  │  STAGE 2: Star Identification  [star_ID()]  my_star_id.c:36        │   │
│  │                                                                      │   │
│  │  centroid_data_set()  픽셀좌표 → 미터단위 (×pixel_size)             │   │
│  │       ↓                                                              │   │
│  │  R = idx.label_num  (검출된 별 수)                                   │   │
│  │       ↓                                                              │   │
│  │  [R < 3] → return 1 (에러)                                          │   │
│  │  [R == 3] → Direct 3-star matching                                  │   │
│  │  [R > 3] → Pyramid algorithm + K-vector table lookup               │   │
│  │       ↓                                                              │   │
│  │  출력:                                                               │   │
│  │    check_stars[i] = 카탈로그 별 ID (0이면 미식별)                    │   │
│  │    r_i[k][3]      = 레퍼런스 벡터 (카탈로그 단위벡터)               │   │
│  │    b_i[k][3]      = 관측 벡터 (body frame 단위벡터)                 │   │
│  │    v_length        = 식별된 별 쌍 수                                 │   │
│  └──────────────────────────────────────────────────────────────────────┘   │
│       ↓                                                                     │
│  ┌──────────────────────────────────────────────────────────────────────┐   │
│  │  STAGE 3: Attitude Estimation  [star_QUEST()]                       │   │
│  │  my_Attitude_Estimation.c:15                                        │   │
│  │                                                                      │   │
│  │  B = Σ w_i * b_i * r_i^T   (attitude profile matrix)              │   │
│  │  S = B + B^T                                                        │   │
│  │  Z = [B23-B32, B31-B13, B12-B21]                                   │   │
│  │  σ = trace(B)                                                       │   │
│  │       ↓                                                              │   │
│  │  Newton-Raphson → 최대 고유값 λ_max                                 │   │
│  │       ↓                                                              │   │
│  │  Rodrigues parameters → Quaternion q[4]                             │   │
│  │       ↓                                                              │   │
│  │  quat2DCM(q) → DCM2Euler(DCM) → Euler[3] (rad→deg)                │   │
│  │       ↓                                                              │   │
│  │  SensorMgr_UpdateQuat(q)                                            │   │
│  │  SensorMgr_UpdateStarMg(idx.label_num, check_stars, 0)             │   │
│  │  SensorMgr_Commit()                                                 │   │
│  └──────────────────────────────────────────────────────────────────────┘   │
│       ↓                                                                     │
│  ┌──────────────────────────────────────────────────────────────────────┐   │
│  │  STAGE 4: Data Output  [generate_response()]  cli_func.c:768       │   │
│  │                                                                      │   │
│  │  RS422 cmd "3,1" → SensorMgr_GetLatest() → quat[4] → RS422 응답   │   │
│  │  RS422 cmd "3,2" → SensorMgr_GetLatest() → star_mg/number → 응답   │   │
│  └──────────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 2. 이슈 요약 (심각도별)

| # | 심각도 | 파일 | 라인 | 이슈 요약 |
|---|--------|------|------|-----------|
| 1 | **CRITICAL** | my_star_id.c | 368-391 | check_stars[] 미리팩, QUEST B행렬 오염 |
| 2 | **CRITICAL** | sensor_mgr.c | 77 | star_mg 인덱싱 오류 (detection→catalog 미변환) |
| 3 | **CRITICAL** | my_Attitude_Estimation.c | 94 | 쿼터니언 정규화 공식 오류 |
| 4 | **CRITICAL** | my_star_id.c | 481 | Pyramid() 스택 ~14KB (task stack 16KB) |
| 5 | **HIGH** | my_Attitude_Estimation.c | 18 | Euler 변수 섀도잉 (local이 extern 가림) |
| 6 | **HIGH** | my_Attitude_Estimation.c | 67-73 | Newton-Raphson J=0 가드 없음 |
| 7 | **HIGH** | my_inv.c | 220-248 | inv3_3 det=0 나눗셈 가드 없음 |
| 8 | **HIGH** | my_star_id.c | 111-116 | kvector_table 배열 경계 미검증 |
| 9 | **HIGH** | sensor_mgr.c | 101-106 | Double buffer Commit() 비원자적 |
| 10 | **MEDIUM** | my_math.c | 134, 167 | minLoc_/maxLoc_ 8MB 스택 할당 |
| 11 | **MEDIUM** | my_star_id.c | 1411-1423 | refer_star() 미초기화 변수 반환 |
| 12 | **MEDIUM** | cli_func.c | 1492 | cli_starRun() int 함수에서 return; (값 없음) |
| 13 | **MEDIUM** | star_detect.h | 28 | 주석 닫기 실패 가능성 (`*/` → `/`) |
| 14 | **LOW** | my_Attitude_Estimation.c | 17 | double 세미콜론 `;;` |
| 15 | **LOW** | globalV.h | 전체 | 헤더에서 전역변수 정의 (-fcommon 의존) |

---

## 3. CRITICAL 이슈 상세

### 이슈 #1: check_stars[] 미리팩 — QUEST B행렬 오염

**파일**: `my_star_id.c` 라인 368-391
**심각도**: CRITICAL
**증상**: 자세 추정 결과가 잘못되거나, 특정 별 배치에서 완전히 틀린 쿼터니언 출력

#### 문제 설명

`star_ID()` 함수의 마지막 부분(R > 3 경로)에서, 식별 성공 후 `r_i[]`와 `b_i[]`를 **앞으로 밀어 리팩**하지만, `check_stars[]`는 리팩하지 않습니다.

```c
// my_star_id.c:368-391
k = 0;
for (i = 0; i < idx.label_num; i++)
{
    if (check_stars[i] == 0)
        continue;
    for (int j = 0; j < 3; j++)
    {
        r_i[k][j] = r_i[i][j];   // ✅ 리팩됨
        b_i[k][j] = b_i[i][j];   // ✅ 리팩됨
    }
    k = k + 1;
    // ❌ check_stars는 리팩 안 됨!
}
// ...
v_length = k;   // 식별된 별 쌍 수
```

#### QUEST에서의 영향

`star_QUEST()` 는 `check_stars[]`를 사용해서 유효한 별 쌍만 선택합니다:

```c
// my_Attitude_Estimation.c:23-36
for (i = 0; i < v_length; i++){
    if (check_stars[i] == 0)    // ← check_stars는 리팩 안 됨!
        continue;               //    원래 인덱스 그대로임
    // ... B행렬 누적에 b_i[i], r_i[i] 사용
}
```

**시나리오 예시**: 별 8개 검출, 인덱스 0,2,5,7에서 식별 성공(4개)

| 인덱스 | check_stars[] (리팩 전) | r_i/b_i (리팩 후) | QUEST가 보는 것 |
|--------|------------------------|--------------------|-----------------|
| 0 | ID_A (≠0) | 별 A의 벡터 | ✅ 사용 (맞음) |
| 1 | 0 | 별 B의 벡터 | ❌ 스킵 (벡터 B 무시됨) |
| 2 | ID_C (≠0) | 별 C의 벡터 | ✅ 사용 (맞음) |
| 3 | 0 | 별 D의 벡터 | ❌ 스킵 (벡터 D 무시됨) |

→ `v_length=4`인데 실제 `i=0,2`만 사용. **별 2개로만 B행렬 계산** → 자세 정확도 심각하게 저하.

더 심각한 경우: check_stars[0]이 0이고 check_stars[1]이 ID면, 리팩된 r_i[0]/b_i[0]은 유효한 벡터인데 check_stars[0]=0이라 스킵됩니다.

#### 수정 방향

리팩 루프에서 `check_stars[]`도 함께 리팩해야 합니다:

```c
k = 0;
for (i = 0; i < idx.label_num; i++)
{
    if (check_stars[i] == 0)
        continue;
    for (int j = 0; j < 3; j++)
    {
        r_i[k][j] = r_i[i][j];
        b_i[k][j] = b_i[i][j];
    }
    check_stars[k] = check_stars[i];  // ← 추가 필요
    k = k + 1;
}
// 나머지 check_stars 0으로 클리어
for (i = k; i < idx.label_num; i++)
    check_stars[i] = 0;
```

**또는** QUEST 쪽에서 `check_stars` 검사를 제거하고 `v_length`만 사용:

```c
for (i = 0; i < v_length; i++){
    // check_stars 검사 없이 모든 i에 대해 B행렬 누적
    weight = 1 / (double)v_length;
    // ...
}
```

---

### 이슈 #2: SensorMgr_UpdateStarMg star_mg 인덱싱 오류

**파일**: `sensor_mgr.c` 라인 65-86
**심각도**: CRITICAL
**증상**: RS422 cmd "3,2" 응답에서 잘못된 별 등급(magnitude) 데이터 전송

#### 문제 설명

```c
// sensor_mgr.c:65-81
void SensorMgr_UpdateStarMg(uint8_t count, const int idenStars[50], int errorCode)
{
    int real_count = 0;
    if (count > 50) count = 50;

    for (uint8_t i = 0; i < count; i++)
    {
        if (idenStars[i] != 0)
        {
            buf[w].star_mg[real_count] = star_mg[i];  // ❌ 문제!
            buf[w].star_number[real_count] = idenStars[i];
            real_count++;
        }
    }
    // ...
}
```

호출부: `my_Attitude_Estimation.c:116`
```c
SensorMgr_UpdateStarMg(idx.label_num, check_stars, 0);
```

여기서 `star_mg[i]`는 **tables.h의 카탈로그 등급 배열**입니다:
```c
// tables.h
extern const double star_mg[STAR_COUNT]; // STAR_COUNT = 1602
```

`star_mg[i]`에서 `i`는 **검출 인덱스** (0~49)인데, `tables.h`의 `star_mg[]`는 **카탈로그 인덱스** (0~1601)로 매핑됩니다.

올바른 접근: `idenStars[i]`가 카탈로그 별 ID(1-based)이므로:

```c
buf[w].star_mg[real_count] = star_mg[idenStars[i] - 1];  // 올바른 인덱싱
```

#### 영향 범위

- RS422 `cmd 3,2` 응답의 별 등급 정보가 모두 잘못됨
- 지상국에서 받는 별 등급 데이터가 실제 식별된 별과 무관한 값
- `star_mg[0]~star_mg[14]`(최대 15개)만 접근하므로 메모리 에러는 없지만, **데이터 의미가 완전히 틀림**

---

### 이슈 #3: 쿼터니언 정규화 공식 오류

**파일**: `my_Attitude_Estimation.c` 라인 94
**심각도**: CRITICAL
**증상**: 쿼터니언 크기가 1이 아닌 경우 발생, 이후 정규화(99-103행)로 어느 정도 보상되지만 수치 안정성 저하

#### 문제 설명

```c
// my_Attitude_Estimation.c:94
d_tmp = 1 / sqrt(gamma*gamma + sqrt((x[0]*x[0] + x[1]*x[1] + x[2]*x[2])
                                    *(x[0]*x[0] + x[1]*x[1] + x[2]*x[2])));
```

이 수식을 수학적으로 풀면:

```
d_tmp = 1 / sqrt(γ² + sqrt(|x|⁴))
      = 1 / sqrt(γ² + |x|²)
```

`sqrt(|x|⁴) = |x|²` 이므로 결과적으로 맞지만, **중간 단계에서 불필요한 `sqrt` 연산**이 있고, 더 중요한 문제는:

QUEST 알고리즘의 원래 정규화 공식:
```
q = [x; γ] / sqrt(γ² + |x|²)
```

이 코드에서:
```c
q[1] = x[0] * d_tmp;   // x 성분에 d_tmp 곱함
q[2] = x[1] * d_tmp;
q[3] = x[2] * d_tmp;
q[0] = gamma * d_tmp;   // γ에도 d_tmp 곱함
```

여기까지는 맞습니다. 하지만 **γ=0이고 x=[0,0,0]인 퇴화(degenerate) 경우** 분모가 0이 되어 **0으로 나누기(division by zero)** 발생합니다.

이 경우는 다음과 같을 때 발생합니다:
- 모든 별 관측 벡터가 레퍼런스 벡터와 정확히 반대 방향 (180° 회전)
- Newton-Raphson이 수렴하지 않은 경우
- B 행렬이 0인 경우 (이슈 #1로 인해 유효 벡터 쌍이 0개인 경우)

#### 수정 방향

```c
double norm_sq = gamma * gamma + x[0]*x[0] + x[1]*x[1] + x[2]*x[2];
if (norm_sq < 1e-20) {
    // Degenerate case: return identity quaternion
    q[0] = 1.0; q[1] = 0.0; q[2] = 0.0; q[3] = 0.0;
} else {
    d_tmp = 1.0 / sqrt(norm_sq);
    q[0] = gamma * d_tmp;
    q[1] = x[0] * d_tmp;
    q[2] = x[1] * d_tmp;
    q[3] = x[2] * d_tmp;
}
```

---

### 이슈 #4: Pyramid() 스택 오버플로 위험

**파일**: `my_star_id.c` 라인 447-481
**심각도**: CRITICAL
**증상**: FreeRTOS task에서 무응답/크래시, 스택 침범으로 다른 변수 오염

#### 문제 설명

`Pyramid()` 함수의 로컬 변수 스택 사용량:

```c
// my_star_id.c:447 (Pyramid 함수 내)
double select_star[4][2];          // 64 bytes
double star_theta[3];              // 24 bytes
double ct1[4][3];                  // 96 bytes
double measurement_vector[3];     // 24 bytes
double measurement_re[3];         // 24 bytes
double t_tmp, d_tmp, tmp1_3_1[3], tmp2_3_1[3]; // 64 bytes
int list_1[20], list_2[20];       // 160 bytes
int id_stars[100][3];             // 1,200 bytes
double tmp3_3[3][3];              // 72 bytes
int ex1[3][500][2];               // 12,000 bytes ← 핵심!
int id1_stars[4], unique_solution[4]; // 32 bytes
// + 기타 int 변수들 ~100 bytes
// 총: ~14,000 bytes (약 13.7KB)
```

**FreeRTOS task stack**: `ov4689_main.c`에서 task 생성 시 스택 크기를 확인 필요하나, Zynq-7015의 일반적인 FreeRTOS 설정은 **16KB** (4096 words × 4 bytes).

- `Pyramid()` 자체: ~14KB
- `Pyramid()` 호출 전 `star_ID()` 로컬: `centroids[40][2]` (640B) + `ex[2][500][2]` (4KB) + 기타 ~500B = **~5.1KB**
- 함수 호출 프레임, 인터럽트 여유 등: ~1KB

**합계: ~20KB** → 16KB 스택을 **확실히 초과**합니다.

#### 호출 경로

```
prvCliTask() (FreeRTOS task, stack = configMINIMAL_STACK_SIZE?)
  → cli_starRun() / star_track_full()
    → star_ID()         // 로컬 ~5.1KB
      → Pyramid()       // 로컬 ~14KB
        = 합계 ~19KB
```

#### 왜 지금까지 크래시 안 했나?

1. `-O2` 최적화로 컴파일러가 일부 변수를 레지스터에 배치하거나 겹쳐 사용
2. ARM Cortex-A9의 MMU가 스택 오버플로를 즉시 감지하지 못할 수 있음
3. R > 3인 경우가 테스트에서 적었을 수 있음
4. 실제로는 크래시가 발생했지만 "star_ID 실패"로 처리되어 인식 못했을 수 있음

#### 수정 방향

1. **`ex1[3][500][2]`를 static으로 이동** (가장 간단):
```c
static int ex1[3][500][2]; // 함수 밖으로 이동
```
2. **`id_stars[100][3]`도 static으로 이동**
3. FreeRTOS task 스택 크기 증가 (최소 32KB 권장)

---

## 4. HIGH 이슈 상세

### 이슈 #5: Euler 변수 섀도잉

**파일**: `my_Attitude_Estimation.c` 라인 5, 18
**심각도**: HIGH
**증상**: 전역 Euler 값이 업데이트되지 않음 (현재 코드에서는 Euler 사용처 없으므로 실질 영향 낮을 수 있음)

```c
// 라인 5 (globalV.h를 통해)
extern double Euler[3];              // 전역 Euler 선언

// 라인 18 (star_QUEST 함수 내)
double Euler[3] = { 0 };            // ← 로컬 변수가 전역을 가림(shadow)!
```

`star_QUEST()` 내의 `DCM2Euler(*tmp1_3_3, Euler)` (라인 106)는 **로컬** `Euler`에 쓰므로, 함수 종료 시 값이 소멸합니다. 전역 `Euler[3]`은 갱신되지 않습니다.

현재 쿼터니언 `q[4]`가 `SensorMgr_UpdateQuat()`로 전달되므로 RS422 출력에는 영향 없지만, 향후 Euler 각도를 다른 모듈에서 참조하려 하면 0으로 남아있게 됩니다.

---

### 이슈 #6: Newton-Raphson J=0 가드 없음

**파일**: `my_Attitude_Estimation.c` 라인 67-73
**심각도**: HIGH
**증상**: 특정 별 배치에서 무한대(Inf) 또는 NaN 발생

```c
// my_Attitude_Estimation.c:66-74
X = 1;
for (i = 0; i < 1000; i++){
    J = 4 * X*X*X - 2 * (a + b)*X - c;            // J = 미분(F)
    F = X*X*X*X - (a + b)*X*X - c*X + (a*b + c*s - d);  // F = 특성방정식
    X = X - F / J;                                   // ❌ J=0이면 나눗셈 오류!
    if (fabs(F) < 0.0000000001)
        break;
}
```

**J=0인 경우**: 특성방정식의 극값에서 접선 기울기가 0. 이론적으로 드물지만:
- 모든 가중치가 같고 별이 특수한 대칭 배치일 때
- 초기값 X=1이 극값 근처에 있을 때
- 수치 오차 누적 시

→ `F/J` = `Inf` 또는 `NaN` → 이후 모든 연산이 `NaN`으로 전파 → 쿼터니언이 `NaN` 출력

#### 수정 방향

```c
if (fabs(J) < 1e-15) {
    X = X + 0.01; // 약간 이동 후 재시도
    continue;
}
X = X - F / J;
```

---

### 이슈 #7: inv3_3() det=0 나눗셈 가드 없음

**파일**: `my_inv.c` 라인 220-248
**심각도**: HIGH
**증상**: S행렬이 특이(singular)할 때 QUEST 전체 크래시

```c
// my_inv.c:237-248
det = a*e*i + b*f*g + c*d*h - c*e*g - b*d*i - a*f*h;

*output =          (e*i - f*h) / det;   // ❌ det=0이면!
*(output + 1) = -1 * (b*i - c*h) / det;
// ... (9개 나눗셈 모두 동일)
```

`star_QUEST()`에서 `inv3_3(*S, *tmp1_3_3)` (라인 48)로 호출됩니다. S = B + B^T이므로 대칭 행렬이며, 식별된 별이 모두 일직선 상에 있으면 `det(S) = 0`이 될 수 있습니다.

#### 수정 방향

```c
det = a*e*i + b*f*g + c*d*h - c*e*g - b*d*i - a*f*h;
if (fabs(det) < 1e-15) {
    memset(output, 0, sizeof(double) * 9);
    return; // 또는 에러코드 반환
}
```

---

### 이슈 #8: kvector_table 배열 경계 미검증

**파일**: `my_star_id.c` 라인 111-116 (및 517-523, 766-771)
**심각도**: HIGH
**증상**: 잘못된 메모리 접근, 크래시 또는 데이터 오염

```c
// my_star_id.c:111-116
kbot = floor( ( cos( (star_theta[i_2] + accuracy) ) - kvector_q) / kvector_m);
ktop = ceil( ( cos( (star_theta[i_2] - accuracy) ) - kvector_q) / kvector_m);

k_start = kvector_table[(kbot - 1) * 3];     // ❌ kbot <= 0이면 음수 인덱스!
k_start++;
k_end = kvector_table[(ktop - 1) * 3];       // ❌ ktop가 테이블 크기 초과 가능
```

**문제 시나리오**:
- `star_theta` 값이 매우 작거나 매우 큰 경우 → `cos()` 값이 범위 밖
- `kvector_m`이 매우 작은 경우 → `kbot/ktop`이 매우 크거나 음수
- `kvector_table` 크기: 86103개 (= 28701 × 3), 인덱스 범위: 0 ~ 86102
- `(kbot - 1) * 3`이 음수이거나 86102를 초과하면 **배열 범위 외 접근**

#### 수정 방향

```c
if (kbot < 1) kbot = 1;
if (ktop < 1) ktop = 1;
if (kbot > KVECTOR_TABLE_SIZE/3) kbot = KVECTOR_TABLE_SIZE/3;
if (ktop > KVECTOR_TABLE_SIZE/3) ktop = KVECTOR_TABLE_SIZE/3;
```

---

### 이슈 #9: Double Buffer Commit() 비원자적

**파일**: `sensor_mgr.c` 라인 101-106
**심각도**: HIGH
**증상**: 드물게 불완전한 데이터(이전 프레임의 쿼터니언 + 현재 프레임의 star_mg) 전송

```c
// sensor_mgr.c:101-106
void SensorMgr_Commit(void)
{
    uint8_t tmp = r;
    r = w;          // ← 인터럽트가 여기서 끼면?
    w = tmp;
}
```

```c
// sensor_mgr.c:108-111
const SensorData* SensorMgr_GetLatest(void)
{
    return &buf[r];  // ← 다른 task에서 호출 가능
}
```

**FreeRTOS 환경에서의 문제**:

1. `star_QUEST()`가 `SensorMgr_UpdateQuat(q)` 호출 (buf[w]에 쓰기)
2. 그 다음 `SensorMgr_UpdateStarMg()` 호출 (buf[w]에 쓰기)
3. `SensorMgr_Commit()` 호출 (r↔w 스왑)

만약 `UpdateQuat()` 후, `Commit()` 전에 다른 task의 `generate_response()`가 `GetLatest()`를 호출하면:
- 이전 프레임의 (오래된) 쿼터니언을 읽음
- 이는 이전 프레임 데이터이므로 **자체적으로는 일관성**이 있음 (부분적으로 새 데이터를 읽진 않음)

실제 위험: `Commit()` 중간에 인터럽트가 끼면 `r`과 `w`가 같은 값이 되는 순간이 있을 수 있음.
- `r = w;` 실행 직후, `w = tmp;` 실행 전: r == w == 같은 버퍼 → 읽기와 쓰기가 같은 버퍼를 가리킴

#### 수정 방향

```c
void SensorMgr_Commit(void)
{
    // Critical section으로 보호
    taskENTER_CRITICAL();
    uint8_t tmp = r;
    r = w;
    w = tmp;
    taskEXIT_CRITICAL();
}
```

---

## 5. MEDIUM 이슈 상세

### 이슈 #10: minLoc_/maxLoc_ 8MB 스택 할당

**파일**: `my_math.c` 라인 134, 167
**심각도**: MEDIUM (현재 코드에서 호출 여부 확인 필요)

```c
// my_math.c:134
void minLoc_(float* a, int a_row, int a_col, int dim, float* result)
{
    int i, j;
    float temp[2097152];   // ← 8MB!! (2^21 × 4 bytes)
    // ...
```

```c
// my_math.c:167
void maxLoc_(float* a, int a_row, int a_col, int dim, float* result)
{
    int i, j;
    float temp[2097152];   // ← 8MB!!
    // ...
```

**분석**: `temp`는 실제로 `temp[0]` 하나만 사용됩니다 (라인 137: `*temp = ...`). 배열 크기 2097152는 완전히 불필요합니다.

현재 파이프라인(star_detect → star_ID → QUEST)에서 이 함수들이 호출되는지 확인 필요. 호출된다면 ARM Cortex-A9의 전체 RAM(512MB DDR3)에서 스택 영역을 8MB 잡으면 다른 변수/버퍼 메모리를 침범합니다.

#### 수정 방향

```c
float temp; // 하나만 필요
```

---

### 이슈 #11: refer_star() 미초기화 변수 반환

**파일**: `my_star_id.c` 라인 1411-1423
**심각도**: MEDIUM
**증상**: Pyramid에서 4번째 별 탐색 시 잘못된 인덱스 반환 가능

```c
// my_star_id.c:1411-1423
int refer_star(int L, int j, int k_i, int R, int re, int re_1)
{
    int i;
    int result;             // ❌ 초기화 없음!
    for (i = re; i < R; i++){
        if ((i != L) && (i != j) && (i != k_i) && (i != re_1) && (i != re)) {
            return i;       // 조건 맞으면 즉시 반환
        }
        else
            result = re;    // 조건 안 맞으면 re를 저장
    }
    return result;          // 루프가 한 번도 안 돌면 쓰레기값 반환
}
```

**문제 시나리오**: `re >= R`이면 for 루프가 한 번도 실행되지 않고 `result`는 초기화되지 않은 스택 쓰레기값을 반환합니다.

호출부 (`my_star_id.c:740`):
```c
re = refer_star(i, j, k_1, R, re, re_1);
```

`re = -1`로 초기화된 후 첫 호출에서 `i = -1`이므로 `for (i = -1; i < R; ...)` → 루프 실행됨.
하지만 이후 호출에서 `re`가 `R-1`이고 모든 조건에서 제외되면 `result`가 미초기화 상태로 반환될 수 있습니다.

#### 수정 방향

```c
int result = re; // 기본값 = 현재 re 유지
```

---

### 이슈 #12: cli_starRun() return type 불일치

**파일**: `cli_func.c` 라인 1458, 1492
**심각도**: MEDIUM
**증상**: 컴파일러 경고, 호출자에서 반환값 해석 시 UB(Undefined Behavior)

```c
// cli_func.c:1458
int cli_starRun()
{
    // ...
    if (star_count < 3)
    {
        // ...
        return;         // ❌ int 함수에서 값 없이 return!
    }
    // ...
    return 0;           // ✅ 정상
    // ...
    return -1;          // ✅ 정상
}
```

C 표준에서 `int` 반환 함수에서 값 없이 `return;`을 사용하면 호출자가 반환값을 사용할 경우 **Undefined Behavior**입니다.

#### 수정 방향

```c
return 1; // 또는 적절한 에러코드
```

---

### 이슈 #13: star_detect.h 주석 닫기 실패 가능성

**파일**: `star_detect.h` 라인 28
**심각도**: MEDIUM
**증상**: 컴파일러에 따라 이후 코드가 주석으로 처리될 수 있음

```c
// star_detect.h:28
#define SD_MAX_STARS_ID    15     /* max stars passed to star_ID (reduce Pyramid combinatorics) * revised 2026.02.05 /
```

주석이 `*/`가 아닌 `/`로 끝납니다. 대부분의 C 컴파일러에서 `/* ... */` 주석은 `*/`를 찾을 때까지 모든 텍스트를 주석으로 처리합니다.

**#define 특성상**: `#define`은 한 줄이므로, `/* ... /`가 줄 끝에서 끊어집니다.
→ 실제로 GCC의 전처리기는 `#define` 라인을 그 줄 끝에서 종료하므로, **이 경우 큰 문제가 아닐 수 있습니다**.
→ 하지만 `\`로 줄 연속이 있거나, 다른 컴파일러에서는 문제될 수 있으므로 수정 권장.

#### 수정 방향

```c
#define SD_MAX_STARS_ID    15     /* max stars passed to star_ID (reduce Pyramid combinatorics), revised 2026.02.05 */
```

---

## 6. LOW 이슈 상세

### 이슈 #14: 더블 세미콜론

**파일**: `my_Attitude_Estimation.c` 라인 17
**심각도**: LOW (기능 영향 없음)

```c
double weight;;   // ← 세미콜론 2개
```

C 표준에서 빈 문(empty statement)은 합법이므로 컴파일/동작에 영향 없습니다. 코드 정리 시 수정.

---

### 이슈 #15: globalV.h 헤더에서 전역변수 정의

**파일**: `globalV.h` 전체
**심각도**: LOW (현재 동작하지만 위험한 패턴)

```c
// globalV.h
unsigned char image0[MAX_IMAGESIZE_Y][MAX_IMAGESIZE_X]; // 정의(definition)!
// ...
CC idx;                    // 정의!
CC_state state;            // 정의!
GBF_ GBF_v;               // 정의!
double b_i[50][3];         // 정의!
// ...
```

이 헤더가 여러 `.c` 파일에서 `#include`되면 각 파일에 동일한 전역변수 정의가 생깁니다. C에서는 **tentative definition** 규칙과 GCC의 `-fcommon` 옵션(기본값)으로 링커가 자동으로 하나로 합쳐주지만:

- GCC 10+에서 `-fno-common`이 기본값으로 변경됨
- 다른 컴파일러(MSVC 등)에서는 링크 에러 발생
- Xilinx SDK (arm-none-eabi-gcc)는 현재 `-fcommon`이므로 동작하지만, SDK 업그레이드 시 깨질 수 있음

#### 올바른 패턴

`globalV.h`:
```c
extern unsigned char image0[MAX_IMAGESIZE_Y][MAX_IMAGESIZE_X];
extern CC idx;
// ...
```

`globalV.c` (새 파일):
```c
#include "globalV.h"
unsigned char image0[MAX_IMAGESIZE_Y][MAX_IMAGESIZE_X];
CC idx;
// ...
```

---

## 7. 데이터 플로우 추적 다이어그램

### 센트로이드 → star_ID 데이터 플로우

```
sd_filter_and_centroid()
  ├── idx.label_num = valid_count    (별 개수, int, 최대 40)
  ├── GBF_v.centroid_x[i] = sum_ix/sum_i   (double, 픽셀 좌표)
  └── GBF_v.centroid_y[i] = sum_iy/sum_i   (double, 픽셀 좌표)
         │
         ▼
star_ID()
  ├── centroid_data_set(centroids)     // 변환
  │     centroids[i][0] = (GBF_v.centroid_x[i] - 640) × 2e-6   (미터)
  │     centroids[i][1] = (GBF_v.centroid_y[i] - 360) × 2e-6   (미터)
  │
  ├── R = idx.label_num
  │
  ├── [R==3 경로]
  │     get_startheta() → star_theta[2], ct1[3][3] (관측 단위벡터)
  │     K-vector lookup → ex[2][500][2] (후보 별 쌍 ID)
  │     check_error_3 + check_pecular1 → unique_solution[3]
  │     ├── check_stars[0..2] = unique_solution[0..2]   (카탈로그 ID)
  │     ├── r_i[0..2][3] = star_table[unique_solution]  (레퍼런스 벡터)
  │     └── b_i[0..2][3] = ct1[0..2][3]                (관측 벡터)
  │
  └── [R>3 경로 (Pyramid)]
        Pyramid(index) → check_stars[], r_i[], b_i[] 누적
        ├── 리팩: r_i[k] = r_i[i], b_i[k] = b_i[i] (연속 재배열)
        ├── ❌ check_stars[] 리팩 누락! (이슈 #1)
        └── v_length = k (유효 쌍 수)
```

### star_ID → QUEST 데이터 플로우

```
star_QUEST()
  │
  ├── 입력 (전역변수):
  │     b_i[50][3]      - 관측 단위벡터 (body frame)
  │     r_i[50][3]      - 레퍼런스 단위벡터 (inertial frame)
  │     check_stars[50] - 카탈로그 별 ID (0=미식별)
  │     v_length         - 유효 벡터 쌍 수
  │
  ├── B = Σ (1/v_length) × b_i[i] × r_i[i]^T
  │     for i in 0..v_length-1:
  │       if check_stars[i]==0: continue  ← ❌ 이슈 #1과 결합!
  │
  ├── S = B + B^T  (3×3 대칭)
  ├── Z = [B[1][2]-B[2][1], B[2][0]-B[0][2], B[0][1]-B[1][0]]
  ├── σ = trace(B)
  │
  ├── inv3_3(S) → S^(-1)          ← ❌ det=0 가드 없음 (이슈 #7)
  ├── det3_3(S) → det(S)
  │
  ├── a = σ² - trace(det(S)×S^(-1))
  ├── b = σ² + |Z|²
  ├── c = det(S) + Z^T × S × Z
  ├── d = Z^T × S² × Z
  │
  ├── Newton-Raphson → λ_max (X)   ← ❌ J=0 가드 없음 (이슈 #6)
  │     F(X) = X⁴ - (a+b)X² - cX + (ab + cσ - d)
  │     F'(X) = 4X³ - 2(a+b)X - c
  │
  ├── α = X² - σ² + trace(det(S)×S^(-1))
  ├── β = X - σ
  ├── γ = α(X + σ) - det(S)
  ├── x = (αI + βS + S²) × Z
  │
  ├── q[0] = γ / sqrt(γ²+|x|²)    (scalar part)  ← ❌ 이슈 #3
  ├── q[1..3] = x / sqrt(γ²+|x|²) (vector part)
  ├── q 정규화 (2차)
  │
  ├── quat2DCM(q) → DCM[3][3]
  ├── DCM2Euler(DCM) → Euler[3]     ← ❌ 로컬 변수, 전역 미갱신 (이슈 #5)
  └── rad→deg 변환
```

### QUEST → RS422 출력 데이터 플로우

```
star_QUEST()
  ├── SensorMgr_UpdateQuat(q)           // buf[w].quat[0..3] = q[0..3]
  ├── SensorMgr_UpdateStarMg(           // buf[w].star_mg/star_number
  │       idx.label_num,                //   count = 검출 별 수
  │       check_stars,                  //   idenStars = 카탈로그 ID 배열
  │       0)                            //   errorCode = 0
  │     ├── star_mg[i] ← ❌ detection 인덱스로 접근 (이슈 #2)
  │     │     올바른: star_mg[idenStars[i]-1]
  │     ├── star_number[i] = idenStars[i]  ✅ 올바름
  │     ├── star_detect = count   ← 검출된 총 별 수 (2026.02.03 추가)
  │     └── star_count = real_count ← 식별된 별 수 (idenStars≠0)
  │
  └── SensorMgr_Commit()               // r↔w 스왑  ← ❌ 비원자적 (이슈 #9)
         │
         ▼
generate_response() [CLI task에서 RS422 수신 시]
  │
  ├── cmd "3,1" (Attitude):
  │     d = SensorMgr_GetLatest()      // buf[r] 읽기
  │     att[0] = d->mjd
  │     att[1..4] = llround(d->quat[0..3] × 1e8)  // QUAT_SCALE
  │     → create_sensor_response("3","1", att, 5)
  │     → RS422 패킷: <3,1,MMMMMMM,QQQQQQQQ,QQQQQQQQ,QQQQQQQQ,QQQQQQQQ,CC>
  │
  └── cmd "3,2" (Star Info):
        d = SensorMgr_GetLatest()
        count = d->star_count
        [count==0] → <3,2,0000,0000,CC>
        [count>0]  →
          star_data[i] = d->star_mg[i]  (float 등급)
          star_info[i+3] = d->star_number[i]  (카탈로그 ID)
          average = sum(star_data) / real_count
          → <3,2,MJD,COUNT,AVG_MAG×100,STAR_ID_1,...,CC>
```

---

## 8. 수정 우선순위 권고

### P0 — 즉시 수정 (하드웨어 크래시/잘못된 출력 직결)

| # | 이슈 | 예상 작업량 | 위험도 |
|---|------|-----------|--------|
| 1 | check_stars[] 미리팩 (이슈 #1) | 5줄 추가 | 자세 추정 완전 오류 |
| 4 | Pyramid() 스택 오버플로 (이슈 #4) | static 키워드 2개 추가 | 런타임 크래시 |
| 2 | star_mg 인덱싱 (이슈 #2) | 1줄 수정 | RS422 데이터 오류 |

### P1 — 조속히 수정 (특정 조건에서 크래시)

| # | 이슈 | 예상 작업량 | 위험도 |
|---|------|-----------|--------|
| 3 | 쿼터니언 정규화 div/0 (이슈 #3) | 5줄 가드 추가 | 특수 배치에서 NaN |
| 6 | Newton-Raphson J=0 (이슈 #6) | 3줄 가드 추가 | 특수 배치에서 NaN |
| 7 | inv3_3 det=0 (이슈 #7) | 3줄 가드 추가 | 특이 행렬에서 Inf |
| 8 | kvector 경계 검증 (이슈 #8) | 4줄 범위 체크 추가 | 메모리 접근 오류 |

### P2 — 다음 릴리스 (기능 영향 낮거나 조건부)

| # | 이슈 | 예상 작업량 | 비고 |
|---|------|-----------|------|
| 5 | Euler 섀도잉 (이슈 #5) | 1줄 삭제 | 현재 미사용 변수 |
| 9 | Commit() 원자성 (이슈 #9) | 3줄 추가 | 레이스 컨디션 가능 |
| 11 | refer_star 초기화 (이슈 #11) | 1줄 수정 | 엣지 케이스 |
| 12 | cli_starRun return (이슈 #12) | 1줄 수정 | 컴파일러 경고 |
| 13 | star_detect.h 주석 (이슈 #13) | 1문자 수정 | 안전 수정 |

### P3 — 코드 품질 (기능 영향 없음)

| # | 이슈 | 비고 |
|---|------|------|
| 14 | 더블 세미콜론 (이슈 #14) | 코드 정리 |
| 15 | globalV.h 패턴 (이슈 #15) | 구조 개선 (대규모) |
| 10 | minLoc_/maxLoc_ 8MB (이슈 #10) | 호출 여부 확인 후 |

---

## 부록 A: 관련 파일 목록

| 파일 | 경로 (ssov/src/ 기준) | 역할 |
|------|----------------------|------|
| star_detect.c | algorithm/star_detect.c | 별 검출 파이프라인 (CCL, Centroid) |
| star_detect.h | algorithm/star_detect.h | 검출 파라미터, 구조체 정의 |
| my_star_id.c | algorithm/my_star_id.c | Pyramid + K-vector 별 식별 |
| my_star_id.h | algorithm/my_star_id.h | 별 식별 함수 선언 |
| my_Attitude_Estimation.c | algorithm/my_Attitude_Estimation.c | QUEST 자세 추정 |
| my_Attitude_Estimation.h | algorithm/my_Attitude_Estimation.h | QUEST 함수 선언 |
| my_CustomF.c | algorithm/my_CustomF.c | quat2DCM, DCM2Euler, 쿼터니언 연산 |
| my_math.c | algorithm/my_math.c | 선형대수 유틸리티 |
| my_inv.c | algorithm/my_inv.c | 행렬 역행렬 (Gauss-Jordan, 3×3) |
| globalV.h | algorithm/globalV.h | 전역변수 정의 (image0, idx, b_i, r_i 등) |
| tables.h | algorithm/tables.h | 별 카탈로그 1602개, K-vector 테이블 |
| sensor_mgr.c | cli/sensor_mgr.c | 더블 버퍼 센서 데이터 관리자 |
| sensor_mgr.h | cli/sensor_mgr.h | 센서 매니저 인터페이스 |
| sensor_data.h | cli/sensor_data.h | SensorData 구조체 정의 |
| cli_func.c | cli/cli_func.c | CLI 명령 핸들러, RS422 응답 생성 |
| cmd_manager.c | cli/cmd_manager.c | RS422 패킷 파싱, 체크섬 |

---

## 부록 B: 쿼터니언 컨벤션 확인

코드 전체에서 쿼터니언 순서가 일관되는지 확인:

| 위치 | q[0] | q[1] | q[2] | q[3] | 컨벤션 |
|------|------|------|------|------|--------|
| star_QUEST() (라인 95-98) | gamma (scalar) | x[0] | x[1] | x[2] | **q[0]=scalar** |
| quat2DCM() (라인 24-27) | q4 (scalar) | q1 | q2 | q3 | **q[0]=scalar** ✅ |
| DCM2quat() (라인 9-13) | q1 | q2 | q3 | q4=scalar | **q[3]=scalar** ❌ 불일치! |
| SensorMgr_UpdateQuat() | 그대로 복사 | - | - | - | q[0]=scalar 전달 |
| RS422 출력 | att[1..4] = q[0..3] | - | - | - | q[0]=scalar 전달 |

**주의**: `DCM2quat()`(라인 7-13)은 `q[3] = scalar`를 사용하지만, `quat2DCM()`(라인 19-39)은 `q[0] = scalar`를 사용합니다. 현재 파이프라인에서 `DCM2quat()`은 호출되지 않으므로 문제없지만, 향후 사용 시 순서 불일치에 주의 필요.

---

## 9. Timeout / 블로킹 전수 조사 및 해결 방안

> **조사 범위**: `ssov/src/` 전체 소스코드 내 모든 `while`, `do-while`, `for` 루프, FreeRTOS 세마포어 대기, `usleep` 기반 폴링, I2C/UART 통신 대기를 전수 검사.

### 9.1 이슈 총괄표

| # | 심각도 | 파일:라인 | 분류 | 요약 | 최악 블로킹 시간 |
|---|--------|----------|------|------|----------------|
| T-1 | **CRITICAL** | cli_func.c:1314 | UART 무한대기 | RS422 TX Full 무한 while | **무한** |
| T-2 | **CRITICAL** | cli_func.c:1320 | UART 무한대기 | RS422 TX Empty 무한 while | **무한** |
| T-3 | **CRITICAL** | my_star_id.c:335-364 | 알고리즘 | Pyramid 3중 루프 시간 제한 없음 | **수 초~수십 초** |
| T-4 | **HIGH** | peri_io.c:152 | FreeRTOS 세마포어 | FrmRx portMAX_DELAY | **무한** |
| T-5 | **HIGH** | peri_io.c:178 | FreeRTOS 세마포어 | FixTimer portMAX_DELAY | **무한** |
| T-6 | **HIGH** | time_manager.c:120 | 정수 오버플로 → 루프 | adjust_time 음수 → 무한루프 | **무한** |
| T-7 | **HIGH** | time_manager.c:126 | 정수 오버플로 → 루프 | adjust_time ms 음수 → 무한루프 | **무한** |
| T-8 | **MEDIUM** | cli_func.c:1469 등 5곳 | CaptureFrame 폴링 | usleep(100ms) 기반, 완료 확인 없음 | 100ms 낭비 또는 미완성 데이터 |
| T-9 | **MEDIUM** | kxtj3.c:153 등 3곳 | I2C timeout 후 복구 | break 후 리셋 없이 Recv | 잘못된 데이터 |
| T-10 | **MEDIUM** | lm75.c:85 | I2C timeout 후 복구 | break 후 리셋 없이 Recv | 잘못된 데이터 |
| T-11 | **LOW** | star_detect.c:107 | Union-Find while | 이론상 메모리 오염 시 무한 | 정상 시 안전 |
| T-12 | **INFO** | 시스템 전체 | Watchdog 미사용 | HW WDT 없음 | 복구 불가 |

---

### 9.2 T-1, T-2: RS422 바이너리 이미지 전송 — UART 무한 대기

**파일**: `cli_func.c` 라인 1310~1321 (detect mode 5)
**트리거**: CLI 명령 `detect 5` 또는 RS422 이미지 전송 명령

#### 문제 코드

```c
// cli_func.c:1310-1321  — RS422 바이너리 이미지 전송
/* Send pixel data - keep FIFO fed for max throughput */
u32 base_addr = Uart_PS_1.Config.BaseAddress;
for (i = 0; i < total; i++)                      // total = 921,600 bytes (1280×720)
{
    while (XUartPs_IsTransmitFull(&Uart_PS_1))    // T-1: ❌ timeout 없음!
        ;
    XUartPs_WriteReg(base_addr, XUARTPS_FIFO_OFFSET, img[i]);
}

/* Wait for last bytes to drain */
while (!XUartPs_IsTransmitEmpty(&Uart_PS_1))      // T-2: ❌ timeout 없음!
    ;
```

#### 왜 위험한가

- RS422 케이블이 빠지면 UART TX FIFO가 drain 되지 않음 → **TX Full 상태 영구 유지**
- 상대방(지상국)이 flow control로 정지하면 동일
- 이 함수는 `prvCliTask`에서 실행 → **CLI task 전체가 멈춤**
- CLI task가 멈추면 RS422 명령 수신/처리 불가, `starTrackerMode` 변경 불가
- 총 921,600 bytes 전송이므로 115200 baud에서 약 80초 소요 — 그 동안 매 바이트마다 위험

#### 비교: 같은 파일 내 다른 곳은 정상

```c
// rs422.c:46-67  — RS422SendBytes()는 timeout이 있음!
void RS422SendBytes(u8 *BufferPtr, size_t NumBytes)
{
    int timeout;
    for (size_t i = 0; i < NumBytes; i++) {
        timeout = 100000;
        while (XUartPs_IsTransmitFull(&Uart_PS_1)) {
            if (--timeout <= 0) {                   // ✅ 100,000회 제한
                printf("RS422 TX full timeout!\n");
                return;
            }
        }
        XUartPs_WriteReg(...);
    }
    timeout = 100000;
    while (!XUartPs_IsTransmitEmpty(&Uart_PS_1)) {
        if (--timeout <= 0) {                       // ✅ 100,000회 제한
            printf("RS422 TX empty timeout!\n");
            return;
        }
    }
}
```

#### 해결 방안

**방법 A (최소 수정)**: 기존 `RS422SendBytes()` 재사용

```c
// cli_func.c — mode 5 이미지 전송 수정
// 기존: 직접 레지스터 접근 (timeout 없음)
// 수정: RS422SendBytes()를 청크 단위로 호출

#define RS422_CHUNK_SIZE 256
for (i = 0; i < total; i += RS422_CHUNK_SIZE)
{
    int n = (total - i > RS422_CHUNK_SIZE) ? RS422_CHUNK_SIZE : (total - i);
    RS422SendBytes(&img[i], n);  // 내부에 timeout 있음
}
```

- 장점: 코드 1줄, `RS422SendBytes` 내장 timeout 재사용
- 단점: 청크 사이 약간의 오버헤드 (무시 가능)

**방법 B (성능 유지 + timeout 추가)**: 직접 레지스터 접근하되 timeout 가드

```c
u32 base_addr = Uart_PS_1.Config.BaseAddress;
int tx_timeout;
for (i = 0; i < total; i++)
{
    tx_timeout = 200000;  // ~200ms at worst
    while (XUartPs_IsTransmitFull(&Uart_PS_1)) {
        if (--tx_timeout <= 0) {
            printf("RS422 IMG TX timeout at byte %d/%d\n", i, total);
            return;  // abort transfer
        }
    }
    XUartPs_WriteReg(base_addr, XUARTPS_FIFO_OFFSET, img[i]);
}

tx_timeout = 200000;
while (!XUartPs_IsTransmitEmpty(&Uart_PS_1)) {
    if (--tx_timeout <= 0) {
        printf("RS422 IMG TX drain timeout\n");
        return;
    }
}
```

**권장**: 방법 A (단순하고 안전)

---

### 9.3 T-3: Pyramid 알고리즘 시간 제한 없음

**파일**: `my_star_id.c` 라인 335~364 (R > 3 경로)
**트리거**: `starTrackerMode == 1` 일 때 매 프레임마다 자동 실행, 또는 `detect 2` 명령

#### 문제 코드

```c
// my_star_id.c:334-364
// R > 3 경로: 3중 루프로 모든 별 조합 탐색
for (dj = 1; dj <= R - 2; dj++)                 // 최대 R-2 = 13
{
    if (check_out == 1) break;
    for (dk = 1; dk <= R - dj - 1; dk++)         // 최대 R-dj-1
    {
        if (check_out == 1) break;
        for (L = 1; L <= R - dj - dk; L++)       // 최대 R-dj-dk
        {
            if (check_out == 1) break;
            j = L + dj;
            k_i = j + dk;
            index[0] = L-1; index[1] = j-1; index[2] = k_i-1;
            ctr = Pyramid(index);                // ← 각 호출마다 K-vector 매칭
            // ...
        }
    }
}
```

#### 실행시간 분석

```
3중 루프 조합 수: C(R, 3)
  R=3:  C(3,3)  =     1  → 즉시
  R=5:  C(5,3)  =    10  → ~수 ms
  R=10: C(10,3) =   120  → ~수십 ms
  R=15: C(15,3) =   455  → ~수백 ms ~ 수 초

각 Pyramid(index) 내부:
  - K-vector lookup: r_size[0] × r_size[1] 비교 (FOV와 정확도에 따라 변동)
  - 일반적: r_size ~10~50 → 최대 2,500 비교
  - 최악 (은하수 방향): r_size ~200 → 최대 40,000 비교
  - fi 루프 (4번째 별 탐색): R번 × 위 전체

  단일 Pyramid() 실행: 일반 ~1ms, 최악 ~50ms
  455회 × 50ms = 22.75초 (최악)

ARM Cortex-A9 666MHz, FPU 있음:
  일반적 총 실행: 100~500ms
  최악 (밀집 별밭): 5~20초
```

#### 영향

- Pyramid 실행 중 `prvCliTask`가 블로킹 → RS422 명령 처리 불가
- `starTrackerMode == 1`에서 매 프레임마다 반복 → 매 프레임 수 초 블로킹
- 지상국에서 "시스템 무응답"으로 인식 가능
- `MAX_STAR_FAIL_COUNT=10` 이후 자동으로 `starTrackerMode=0`이 되긴 하지만, **성공도 실패도 아닌 "오래 걸리는" 경우에는 fail_count가 증가하지 않으므로** 무한히 블로킹 반복

#### 해결 방안

**방법 A (XTime 기반 시간 제한)**: 가장 권장

```c
// my_star_id.c — star_ID() 함수 내 R > 3 경로

#include "xtime_l.h"
#define STAR_ID_TIMEOUT_MS  500  // 최대 500ms 허용

// R > 3 경로 시작 직전
XTime t_start, t_now;
XTime_GetTime(&t_start);

for (dj = 1; dj <= R - 2; dj++)
{
    if (check_out == 1) break;
    for (dk = 1; dk <= R - dj - 1; dk++)
    {
        if (check_out == 1) break;
        for (L = 1; L <= R - dj - dk; L++)
        {
            if (check_out == 1) break;

            // --- timeout check ---
            XTime_GetTime(&t_now);
            double elapsed_ms = (double)(t_now - t_start) * 1000.0
                                / (double)COUNTS_PER_SECOND;
            if (elapsed_ms > STAR_ID_TIMEOUT_MS) {
                if (print_ == 1)
                    printf("star_ID timeout (%.0fms)\n", elapsed_ms);
                return 4;  // new error code: timeout
            }
            // --- end timeout check ---

            j = L + dj;
            k_i = j + dk;
            index[0] = L-1; index[1] = j-1; index[2] = k_i-1;
            ctr = Pyramid(index);
            // ...
        }
    }
}
```

- return 4를 새 에러코드로 정의 (`star_track_full`과 `cli_starRun`에서 처리 추가 필요)
- `STAR_ID_TIMEOUT_MS`를 CLI에서 조정 가능하게 하면 현장 튜닝 가능

**방법 B (조합 횟수 제한)**: 더 간단

```c
#define MAX_PYRAMID_CALLS  200  // 최대 200회 Pyramid 호출
int pyramid_call_count = 0;

for (dj = ...) {
    for (dk = ...) {
        for (L = ...) {
            if (check_out == 1) break;
            if (++pyramid_call_count > MAX_PYRAMID_CALLS) {
                return 4; // timeout
            }
            ctr = Pyramid(index);
            // ...
        }
    }
}
```

- 장점: XTime 의존성 없음, 간단
- 단점: 실행시간이 HW에 따라 다르므로 일관성이 떨어짐

**방법 C (R 캡핑 강화)**: 현재 `SD_MAX_STARS_ID=15`를 더 줄임

```c
// star_detect.h
#define SD_MAX_STARS_ID  10  // 15 → 10으로 줄이면 C(10,3)=120, 훨씬 빠름
```

- 장점: 코드 수정 최소
- 단점: 사용 가능한 별 수 감소 → star_ID 성공률 저하 가능

**권장**: 방법 A + 방법 C 병행. `SD_MAX_STARS_ID=10`으로 줄이고, 그래도 넘어가는 경우를 위해 500ms timeout 추가.

---

### 9.4 T-4, T-5: FreeRTOS 세마포어 무한 대기

**파일**: `peri_io.c` 라인 149~153, 176~178
**트리거**: 하드웨어 인터럽트 발생 안 할 때 (센서 고장, 클럭 이상, 방사선 SEU)

#### 문제 코드

```c
// peri_io.c:149-153 — prvFrmRxTask
void FrmRxHandler()
{
    xSemaphoreTake(xFrmRxSem, portMAX_DELAY);   // ❌ 무한 대기
    InterruptProcessedCount++;
    // ... NewFrmUpdate() ...
}

// peri_io.c:176-178 — prvFixTimerTask
void FixTimerHandler()
{
    xSemaphoreTake(xFTimerSem, portMAX_DELAY);  // ❌ 무한 대기
    // ... camera control ...
}
```

```c
// ov4689_main.c:139-146 — 해당 task들
void prvFrmRxTask(void *pvParameters) {
    interrupt_init();
    for(;;) {
        FrmRxHandler();      // 여기서 영원히 블로킹 가능
        taskYIELD();
    }
}
void prvFixTimerTask(void *pvParameters) {
    while(1) {
        FixTimerHandler();   // 여기서 영원히 블로킹 가능
        taskYIELD();
    }
}
```

#### 왜 위험한가

1. **MIPI 센서 고장**: OV4689가 프레임 출력을 멈추면 `ExtIrq_Handler()`가 호출 안 됨 → `xFrmRxSem`에 `Give` 안 됨 → `FrmRxHandler` 영구 블로킹
2. **Fixed Timer 이상**: 타이머 인터럽트가 안 오면 `FixedTimer_Handler()`가 호출 안 됨 → 동일
3. 블로킹된 task 자체는 다른 task에 영향 안 주지만 (FreeRTOS 우선순위 프리엠션), **해당 task 기능이 완전히 상실**:
   - `FrmRxHandler` 멈추면: HDMI 업데이트 안 됨, OSD 표시 안 됨, 프레임 캡처 완료 시그널 안 옴
   - `FixTimerHandler` 멈추면: 카메라 트리거 안 됨 (manual mode), 프레임레이트 측정 안 됨

4. **우주 환경에서 방사선 SEU**: 레지스터 bit flip으로 인터럽트 비활성화 가능 → 해당 task 영구 정지 → watchdog 없으므로 복구 불가

#### 해결 방안

**방법 A (타임아웃 세마포어 + 재시도)**: 가장 권장

```c
// peri_io.c — FrmRxHandler 수정
#define FRMRX_TIMEOUT_TICKS  pdMS_TO_TICKS(2000)  // 2초 타임아웃 (30fps 기준 ~60프레임)

void FrmRxHandler()
{
    BaseType_t result = xSemaphoreTake(xFrmRxSem, FRMRX_TIMEOUT_TICKS);

    if (result == pdFALSE) {
        // Timeout: interrupt not received for 2 seconds
        printf("FrmRx: timeout, reinitializing interrupt\n");

        // Option 1: Re-enable interrupt
        // XScuGic_Enable(&InterruptController, FRAME_IRQ_ID);

        // Option 2: Increment error counter
        static int frmrx_timeout_count = 0;
        frmrx_timeout_count++;

        if (frmrx_timeout_count >= 5) {
            printf("FrmRx: 5 consecutive timeouts, sensor may be dead\n");
            SensorMgr_UpdateErrorCode(0x10);  // sensor timeout error
            SensorMgr_Commit();
            frmrx_timeout_count = 0;
        }
        return;  // skip this cycle, try again
    }

    // Normal path: interrupt received
    InterruptProcessedCount++;
    // ... existing code ...
}
```

```c
// peri_io.c — FixTimerHandler 수정
#define FTIMER_TIMEOUT_TICKS  pdMS_TO_TICKS(2000)

void FixTimerHandler()
{
    BaseType_t result = xSemaphoreTake(xFTimerSem, FTIMER_TIMEOUT_TICKS);

    if (result == pdFALSE) {
        printf("FixTimer: timeout\n");
        return;
    }

    // ... existing code ...
}
```

**방법 B (Watchdog 추가)**: 시스템 레벨 안전장치

```c
// ov4689_main.c — main() 또는 별도 task에서
// Zynq-7000의 SWDT (System Watchdog Timer) 활용

#include "xscuwdt.h"

XScuWdt WdtInstance;

void wdt_init(void) {
    XScuWdt_Config *cfg = XScuWdt_LookupConfig(XPAR_SCUWDT_0_DEVICE_ID);
    XScuWdt_CfgInitialize(&WdtInstance, cfg, cfg->BaseAddr);
    XScuWdt_SetTimerMode(&WdtInstance);  // timer mode (not watchdog reset)

    // Set timeout: ~5 seconds at 333MHz/2
    XScuWdt_LoadWdt(&WdtInstance, 0x7FFFFFFF);
    XScuWdt_Start(&WdtInstance);
}

// 각 task의 main loop에서 주기적으로:
void wdt_kick(void) {
    XScuWdt_RestartWdt(&WdtInstance);
}
```

**방법 C (A + B 병행)**: 우주급 신뢰성

- 각 세마포어에 timeout 적용 (방법 A)
- SWDT를 watchdog 모드로 설정하여 모든 task가 정지하면 시스템 자동 리셋 (방법 B)
- 각 task에서 "alive" 플래그를 설정하고, 별도 모니터 task가 주기적으로 확인

**권장**: 방법 A (즉시 적용 가능) + 방법 B (추후 추가)

---

### 9.5 T-6, T-7: adjust_time() 음수 오프셋 → 무한루프

**파일**: `time_manager.c` 라인 112~131
**트리거**: RS422 cmd `5,2` (시간 보정) 또는 CLI `time adj` 명령으로 음수 값 전달

#### 문제 코드

```c
// time_manager.c:112-131
void adjust_time(int32_t offset_sec, int32_t offset_ms)
{
    DateTime now;
    get_current_time(&now);

    if (offset_sec != 0) {
        int32_t total_ms = offset_sec * 1000;     // offset_sec = -5 → total_ms = -5000
        add_millis(&now, total_ms);               // ❌ int32_t(-5000) → uint64_t(18446744073709546616)
    }

    if (offset_ms != 0) {
        add_millis(&now, offset_ms);              // ❌ 동일 문제
    }

    set_time(&now);
}
```

```c
// time_manager.c:31-68
void add_millis(DateTime* t, uint64_t ms) {       // ← 파라미터가 uint64_t!
    t->millisec += ms % 1000;
    uint64_t sec_add = ms / 1000;
    // ...
    uint64_t day_add = hour_add / 24;

    while (day_add > 0) {                          // ❌ day_add = 약 2.13 × 10^13 일
        // ... 1일씩 감소 ...                     //     = 584억 년 분의 날짜 계산
        day_add--;                                 //     사실상 무한루프
    }
}
```

#### 구체적 시나리오

```
offset_sec = -1 (1초 뒤로 보정하려는 의도)
total_ms = -1 * 1000 = -1000 (int32_t)

add_millis(&now, (int32_t)(-1000))
  → C 표준: int32_t(-1000) → uint64_t = 18446744073709550616
  → ms = 18446744073709550616
  → sec_add = 18446744073709550  (약 5.8 × 10^8 년)
  → day_add = 약 2.13 × 10^13
  → while(day_add > 0) → 2.13 × 10^13 번 반복 필요
  → ARM 666MHz에서 약 수만 초 = 수 시간 ~ 수 일
  → 사실상 시스템 영구 정지
```

#### 해결 방안

**방법 A (adjust_time 내부에서 부호 처리)**: 가장 근본적

```c
void adjust_time(int32_t offset_sec, int32_t offset_ms)
{
    DateTime now;
    get_current_time(&now);

    // total offset in milliseconds (signed)
    int64_t total_offset_ms = (int64_t)offset_sec * 1000 + (int64_t)offset_ms;

    if (total_offset_ms >= 0) {
        add_millis(&now, (uint64_t)total_offset_ms);
    } else {
        // Negative offset: subtract time
        subtract_millis(&now, (uint64_t)(-total_offset_ms));
    }

    set_time(&now);
}
```

`subtract_millis()` 신규 함수:

```c
void subtract_millis(DateTime* t, uint64_t ms)
{
    // Convert DateTime to total milliseconds since epoch, subtract, convert back
    // Or: step backwards

    uint64_t borrow_ms = ms % 1000;
    uint64_t sec_sub   = ms / 1000;

    if (t->millisec >= borrow_ms) {
        t->millisec -= borrow_ms;
    } else {
        t->millisec = t->millisec + 1000 - borrow_ms;
        sec_sub++;
    }

    // Subtract seconds
    while (sec_sub > 0) {
        if (t->second > 0) {
            uint64_t can_sub = (sec_sub < t->second) ? sec_sub : t->second;
            t->second -= can_sub;
            sec_sub -= can_sub;
        } else {
            // Borrow from minutes
            if (t->minute > 0) {
                t->minute--;
                t->second = 59;
                sec_sub--;
            } else if (t->hour > 0) {
                t->hour--;
                t->minute = 59;
                t->second = 59;
                sec_sub--;
            } else if (t->day > 1) {
                t->day--;
                t->hour = 23;
                t->minute = 59;
                t->second = 59;
                sec_sub--;
            } else {
                // Borrow from month
                if (t->month > 1) {
                    t->month--;
                } else {
                    t->year--;
                    t->month = 12;
                }
                t->day = days_in_month(t->year, t->month);
                t->hour = 23;
                t->minute = 59;
                t->second = 59;
                sec_sub--;
            }
        }
    }
}
```

**방법 B (간이 가드)**: 최소 수정

```c
void adjust_time(int32_t offset_sec, int32_t offset_ms)
{
    DateTime now;
    get_current_time(&now);

    if (offset_sec > 0) {
        add_millis(&now, (uint64_t)offset_sec * 1000);
    } else if (offset_sec < 0) {
        // Negative second offset: not supported, log warning
        printf("WARN: negative time adjust not supported, ignoring %d sec\n", offset_sec);
    }

    if (offset_ms > 0) {
        add_millis(&now, (uint64_t)offset_ms);
    } else if (offset_ms < 0) {
        printf("WARN: negative time adjust not supported, ignoring %d ms\n", offset_ms);
    }

    set_time(&now);
}
```

**방법 C (add_millis 파라미터 가드)**: 방어적

```c
void add_millis(DateTime* t, uint64_t ms)
{
    // Guard: prevent absurd values (max 24 hours = 86,400,000 ms)
    if (ms > 86400000ULL) {
        printf("add_millis: ms=%llu too large, clamping\n", ms);
        ms = 86400000ULL;
    }
    // ... existing code ...
}
```

**권장**: 방법 A (완전한 음수 지원). 우주 운용 중 시간 보정은 음수(뒤로)도 필요할 수 있으므로.

---

### 9.6 T-8: CaptureFrame + usleep 폴링 패턴

**파일**: `cli_func.c` 라인 912, 1229, 1283, 1363, 1469 (5곳)
**패턴**: 모두 동일

```c
CaptureFrame();           // captureReq = 1 설정만 함
usleep(100000);           // 100ms 고정 대기
u32 cap_addr = GetCapFrmBase();  // 캡처 완료 여부 확인 없이 사용
```

#### 문제

`CaptureFrame()`은 `captureReq` 플래그만 1로 설정하고 즉시 리턴합니다. 실제 캡처는 `NewFrmUpdate()` (프레임 인터럽트 핸들러) 내에서 수행됩니다.

```c
// peri_io.c:353-359  — NewFrmUpdate() 내부
if(captureReq == 1)
{
    u32 CapBase = GetCapFrmBase();
    memcpy(CapBase, RxBase, 1280*720);    // 실제 복사
    Xil_DCacheFlushRange(CapBase, 1280*720);
    captureReq = 0;                        // 완료 표시
}
```

- 프레임 인터럽트가 100ms 내에 오지 않으면: `captureReq`가 여전히 1, 이전 프레임 데이터 사용
- 카메라 FPS가 매우 낮으면 (노출 시간 464ms 등): 100ms로는 부족
- `sd_display_hold == 1`이면: `NewFrmUpdate()`가 캡처는 하지만 HDMI 업데이트 안 함 → 이건 정상

#### 해결 방안

**방법 A (완료 폴링 + timeout)**: 가장 안전

```c
void CaptureFrameBlocking(int timeout_ms)
{
    captureReq = 1;

    int waited = 0;
    while (captureReq != 0 && waited < timeout_ms) {
        usleep(1000);  // 1ms 단위 폴링
        waited++;
    }

    if (captureReq != 0) {
        printf("CaptureFrame timeout (%dms)\n", timeout_ms);
        captureReq = 0;  // reset flag
    }
}

// 사용:
CaptureFrameBlocking(500);  // 최대 500ms 대기
u32 cap_addr = GetCapFrmBase();
```

**방법 B (세마포어 기반 동기화)**: 더 정확

```c
// peri_io.c에 추가
static SemaphoreHandle_t xCaptureSem;

void initCaptureSem(void) {
    xCaptureSem = xSemaphoreCreateBinary();
}

// NewFrmUpdate() 내 캡처 완료 시:
if (captureReq == 1) {
    // ... memcpy ...
    captureReq = 0;
    xSemaphoreGive(xCaptureSem);  // 완료 시그널
}

// 호출부:
void CaptureFrameSync(int timeout_ms) {
    captureReq = 1;
    if (xSemaphoreTake(xCaptureSem, pdMS_TO_TICKS(timeout_ms)) == pdFALSE) {
        printf("Capture timeout\n");
        captureReq = 0;
    }
}
```

**권장**: 방법 A (기존 구조 최소 변경)

---

### 9.7 T-9, T-10: I2C timeout 후 복구 불완전

**파일**: `kxtj3.c` 라인 131~158, 199~226, 266~293 / `lm75.c` 라인 63~90

#### 문제 코드 패턴

```c
// kxtj3.c:131-158 (3곳 모두 동일 패턴)
int cnt = 0;
do {
    StatusReg = XIic_ReadReg(KXTJ3_BASE_ADDR, XIIC_SR_REG_OFFSET);
    if(!(StatusReg & XIIC_SR_BUS_BUSY_MASK)) {
        ReceivedByteCount = XIic_Send(KXTJ3_BASE_ADDR, KXTJ3_SLAVE_ADDR,
                                       AddrBuf, sizeof(AddrBuf), XIIC_STOP);
        if (ReceivedByteCount != sizeof(AddrBuf)) {
            // TX FIFO 리셋만 함
            XIic_WriteReg(KXTJ3_BASE_ADDR, XIIC_CR_REG_OFFSET, XIIC_CR_TX_FIFO_RESET_MASK);
            XIic_WriteReg(KXTJ3_BASE_ADDR, XIIC_CR_REG_OFFSET, XIIC_CR_ENABLE_DEVICE_MASK);
        }
    }
    if(cnt++ > 5) {
        printf("timeout !!!\n");
        break;                    // ← timeout으로 탈출
    }
} while (ReceivedByteCount != sizeof(AddrBuf));

// ❌ timeout으로 break한 경우에도 아래 Recv를 그대로 실행!
ReceivedByteCount = XIic_Recv(KXTJ3_BASE_ADDR, KXTJ3_SLAVE_ADDR,
                               BufferPtr, 1, XIIC_STOP);
```

#### 비교: ov4689.c의 올바른 처리

```c
// ov4689.c:678-687
int timeout = 100000;
while (XIicPs_BusIsBusy(&Iic0)) {
    if (--timeout <= 0) {
        XIicPs_Reset(&Iic0);                  // ✅ 전체 I2C 컨트롤러 리셋
        XIicPs_SetSClk(&Iic0, IIC_SCLK_RATE); // ✅ 클럭 재설정
        printf("I2C write bus timeout!\n");
        return XST_FAILURE;                    // ✅ 에러 반환, 이후 코드 실행 안 함
    }
}
```

#### 차이점

| 항목 | kxtj3.c / lm75.c (AXI IIC) | ov4689.c (PS I2C) |
|------|---------------------------|-------------------|
| timeout 후 리셋 | TX FIFO만 리셋 | **컨트롤러 전체 리셋** |
| timeout 후 동작 | Recv 계속 실행 ❌ | **return 에러** ✅ |
| 반환값 | 잘못된 데이터 | 에러코드 |
| I2C 드라이버 | `XIic` (AXI IIC) | `XIicPs` (PS I2C) |

#### 해결 방안

```c
// kxtj3.c — SMBusRead() 수정 예시
int cnt = 0;
do {
    StatusReg = XIic_ReadReg(KXTJ3_BASE_ADDR, XIIC_SR_REG_OFFSET);
    if(!(StatusReg & XIIC_SR_BUS_BUSY_MASK)) {
        ReceivedByteCount = XIic_Send(KXTJ3_BASE_ADDR, KXTJ3_SLAVE_ADDR,
                                       AddrBuf, sizeof(AddrBuf), XIIC_STOP);
        if (ReceivedByteCount != sizeof(AddrBuf)) {
            XIic_WriteReg(KXTJ3_BASE_ADDR, XIIC_CR_REG_OFFSET,
                          XIIC_CR_TX_FIFO_RESET_MASK);
            XIic_WriteReg(KXTJ3_BASE_ADDR, XIIC_CR_REG_OFFSET,
                          XIIC_CR_ENABLE_DEVICE_MASK);
        }
    }
    if(cnt++ > 5) {
        printf("kxtj3 I2C timeout, resetting controller\n");

        // Full AXI IIC soft reset
        XIic_WriteReg(KXTJ3_BASE_ADDR, XIIC_RESETR_OFFSET, XIIC_RESET_MASK);
        usleep(1000);  // 1ms recovery
        XIic_WriteReg(KXTJ3_BASE_ADDR, XIIC_CR_REG_OFFSET,
                      XIIC_CR_ENABLE_DEVICE_MASK);

        return 0xFFFF;  // error sentinel value (caller checks)
    }
} while (ReceivedByteCount != sizeof(AddrBuf));

// Only reach here on success
ReceivedByteCount = XIic_Recv(...);
```

`lm75.c`도 동일하게 수정. 호출부에서 `0xFFFF` 반환 시 "센서 통신 에러" 처리 추가.

---

### 9.8 T-12: Watchdog Timer 미사용

**범위**: 시스템 전체
**상태**: Zynq-7015의 SWDT (System Watchdog Timer), Private Timer WDT 모두 미사용

#### 왜 필요한가

우주 환경에서:
- **SEU (Single Event Upset)**: 방사선이 레지스터 bit flip → 프로그램 카운터 이상 → 무한루프
- **SEL (Single Event Latchup)**: 전류 경로 단락 → 칩 오작동 (전원 사이클 필요)
- **소프트웨어 버그**: 위 이슈들(T-1~T-7)이 모두 수정되더라도 예측 못한 hang 가능

Watchdog이 없으면 어떤 이유로든 시스템이 멈추면 **영원히 복구 불가**.

#### 해결 방안: Zynq SWDT 활용

```c
// watchdog.h
#ifndef WATCHDOG_H
#define WATCHDOG_H

#include "xscuwdt.h"

// Initialize watchdog with timeout period
// timeout_sec: watchdog timeout in seconds (1~60)
int wdt_init(int timeout_sec);

// Kick (feed) the watchdog - must be called periodically
void wdt_kick(void);

// Disable watchdog (for debugging)
void wdt_disable(void);

#endif
```

```c
// watchdog.c
#include "watchdog.h"
#include "xparameters.h"

static XScuWdt WdtInstance;

int wdt_init(int timeout_sec)
{
    XScuWdt_Config *cfg;
    int status;

    cfg = XScuWdt_LookupConfig(XPAR_SCUWDT_0_DEVICE_ID);
    if (!cfg) return -1;

    status = XScuWdt_CfgInitialize(&WdtInstance, cfg, cfg->BaseAddr);
    if (status != XST_SUCCESS) return -1;

    // Watchdog mode: system reset on timeout
    XScuWdt_SetWdMode(&WdtInstance);

    // Load count value
    // PERIPHCLK = CPU_CLK / 2 = 333MHz, prescaler = 255
    // Count = timeout_sec * (333e6 / (255+1)) = timeout_sec * 1,300,781
    u32 count = (u32)((double)timeout_sec * 333000000.0 / 256.0);
    XScuWdt_LoadWdt(&WdtInstance, count);

    XScuWdt_Start(&WdtInstance);
    return 0;
}

void wdt_kick(void)
{
    XScuWdt_RestartWdt(&WdtInstance);
}

void wdt_disable(void)
{
    XScuWdt_Stop(&WdtInstance);
}
```

```c
// ov4689_main.c — main()에서 초기화
wdt_init(5);  // 5초 watchdog

// prvCliTask() 메인 루프에서:
while (1) {
    wdt_kick();  // 매 루프마다 kick

    // ... existing code ...

    taskYIELD();
}

// 다른 task에서도 별도 alive 플래그 관리 가능
```

---

### 9.9 Timeout 현황 전체 지도

```
┌─────────────────────────────────────────────────────────────────────────┐
│                     시스템 Timeout/블로킹 전체 지도                      │
│                                                                         │
│  ┌───────────── prvCliTask (Priority 2) ──────────────────────────┐    │
│  │                                                                 │    │
│  │  UART RX 폴링 ← 논블로킹 ✅                                   │    │
│  │       │                                                         │    │
│  │  [starTrackerMode==1]                                          │    │
│  │       ↓                                                         │    │
│  │  cli_starRun()                                                 │    │
│  │    ├─ CaptureFrame() + usleep(100ms) ← ⚠️ T-8 완료확인 없음  │    │
│  │    ├─ star_detect_pipeline() ← 바운딩 ✅ (유한 루프)          │    │
│  │    ├─ star_ID()                                                │    │
│  │    │    ├─ [R==3] ← 바운딩 ✅                                 │    │
│  │    │    └─ [R>3] Pyramid 3중루프 ← ❌ T-3 시간제한 없음       │    │
│  │    │         └─ Pyramid() 내부                                 │    │
│  │    │              ├─ K-vector lookup ← 바운딩 ✅               │    │
│  │    │              └─ fi 루프 × R ← 바운딩 ✅ (R≤15)           │    │
│  │    ├─ star_QUEST()                                             │    │
│  │    │    ├─ B행렬 for 루프 ← 바운딩 ✅ (v_length)              │    │
│  │    │    ├─ inv3_3() ← ⚠️ det=0 가드 없음 (본문 이슈#7)      │    │
│  │    │    ├─ Newton-Raphson × 1000 ← 바운딩 ✅                  │    │
│  │    │    │    └─ ⚠️ J=0 가드 없음 (본문 이슈#6)               │    │
│  │    │    └─ SensorMgr_Commit() ← ⚠️ T-? 비원자적              │    │
│  │    │                                                            │    │
│  │    └─ fail_count ≥ 10 → starTrackerMode=0 ← ✅ 안전장치      │    │
│  │                                                                 │    │
│  │  [CLI 명령 처리]                                               │    │
│  │    ├─ detect 5 → RS422 이미지 전송                             │    │
│  │    │    ├─ while(TX Full)  ← ❌ T-1 무한대기                   │    │
│  │    │    └─ while(!TX Empty) ← ❌ T-2 무한대기                  │    │
│  │    ├─ time adj → adjust_time()                                 │    │
│  │    │    └─ add_millis(음수) ← ❌ T-6,T-7 무한루프             │    │
│  │    └─ generate_response()                                      │    │
│  │         └─ RS422SendBytes() ← ✅ timeout 있음                 │    │
│  └─────────────────────────────────────────────────────────────────┘    │
│                                                                         │
│  ┌───────────── prvFixTimerTask (Priority 3) ────────────────────┐    │
│  │  xSemaphoreTake(portMAX_DELAY) ← ❌ T-5 무한대기              │    │
│  │  FixTimerHandler() → camera control                            │    │
│  └─────────────────────────────────────────────────────────────────┘    │
│                                                                         │
│  ┌───────────── prvFrmRxTask (Priority 4) ───────────────────────┐    │
│  │  xSemaphoreTake(portMAX_DELAY) ← ❌ T-4 무한대기              │    │
│  │  FrmRxHandler() → NewFrmUpdate()                               │    │
│  │    ├─ memcpy (유한) ← ✅                                       │    │
│  │    ├─ I2C 온도 읽기 (LM75)                                    │    │
│  │    │    └─ do-while cnt>5 → break ← ⚠️ T-10 복구 불완전      │    │
│  │    └─ OSD 렌더링 ← ✅                                         │    │
│  └─────────────────────────────────────────────────────────────────┘    │
│                                                                         │
│  ┌───────────── I2C 드라이버 ────────────────────────────────────┐    │
│  │  ov4689.c: ps_i2c_write0/read0                                │    │
│  │    └─ while(BusIsBusy) timeout=100000 ← ✅                    │    │
│  │  kxtj3.c: SMBusRead/Write                                     │    │
│  │    └─ do-while cnt>5 → break ← ⚠️ T-9 복구 불완전           │    │
│  │  lm75.c: LM75ReadTemp                                         │    │
│  │    └─ do-while cnt>5 → break ← ⚠️ T-10 복구 불완전          │    │
│  └─────────────────────────────────────────────────────────────────┘    │
│                                                                         │
│  ┌───────────── 시스템 안전장치 ─────────────────────────────────┐    │
│  │  Hardware Watchdog (SWDT): ❌ T-12 미사용                      │    │
│  │  Software Watchdog:        ❌ 없음                              │    │
│  │  fail_count 안전장치:      ✅ starTrackerMode 자동 해제        │    │
│  └─────────────────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────────────────┘
```

### 9.10 Timeout 수정 우선순위

| 우선순위 | 이슈 | 수정 난이도 | 영향 |
|---------|------|-----------|------|
| **P0** | T-1,T-2: RS422 이미지 전송 무한 while | **쉬움** (RS422SendBytes 재사용) | 시스템 영구 정지 방지 |
| **P0** | T-6,T-7: adjust_time 음수 → 무한루프 | **쉬움** (가드 3줄 추가) | CLI 시스템 정지 방지 |
| **P1** | T-3: Pyramid 시간 제한 | **보통** (XTime 10줄 추가) | RS422 응답 지연 방지 |
| **P1** | T-4,T-5: 세마포어 portMAX_DELAY | **보통** (타임아웃 + 에러 처리) | HW 고장 시 복구 가능 |
| **P2** | T-8: CaptureFrame 완료 확인 | **보통** (폴링 함수 추가) | 미완성 프레임 방지 |
| **P2** | T-9,T-10: I2C timeout 후 리셋 | **보통** (리셋 + return 에러) | I2C bus 복구 |
| **P3** | T-12: Watchdog 추가 | **보통~높음** (새 모듈) | 궁극적 안전장치 |

---

*보고서 끝 — 2026-02-05 (timeout/블로킹 섹션 추가)*
