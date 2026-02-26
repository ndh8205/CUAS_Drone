# IAC 2026 세션 선택 검토

- **마감**: 2026.02.28 (23:59 CET) — 2일 남음
- **대회**: 77th IAC, Antalya, Turkiye, 2026.10.5–9
- **논문 제목**: Control Moment Gyroscope-Augmented Attitude Control for Next-Generation Mars Rotorcraft

---

## 후보 세션 비교

### Option 1: A3.3A — Mars Exploration: Missions Current and Future

> "Current results from ongoing Mars missions and the designs for proposed Mars missions."

| 항목 | 내용 |
|------|------|
| **Symposium** | A3 — IAF Space Exploration Symposium |
| **Type** | Oral |
| **Co-Chairs** | Vincenzo Giorgio (Thales Alenia Space, Italy), Pierre W. Bousquet (CNES, France) |
| **장점** | 화성 회전익기 논문 최다 제출 세션 (IAC-22~25에서 5편 이상 확인). Scope 논란 없음 |
| **단점** | "미션 설계" 관점 중심이라, CMG 제어 아키텍처의 기술적 novelty가 부각되기 어려움 |

**이 세션에 제출된 화성 회전익기/항공기 논문 선례:**

| 연도 | 논문 코드 | 제목 | 저자 |
|------|----------|------|------|
| IAC-24 | IAC-24,A3,3A,16,x85796 | "Systematic Selection of the Next Generation Martian Rotorcraft Configurations" | Vishal Youhanna, Leonard Felicetti, Dmitry Ignatyev (Cranfield University, UK) |
| IAC-24 | IAC-24,A3,3A,13,x85408 | "Analysis of Design Concepts for Mars Unmanned Aerial Vehicles" | Wei Han, Manuel Carreno Ruiz, Domenic D'Ambrosio (Politecnico di Torino, Italy) |
| IAC-24 | IAC-24,A3,3A,14,x83593 | "Feasible Mission Design of a Martian Airship and Verification with a Trajectory Control Simulation" | Koki Kimura, Lina Kuhlmann, Kelly Touzeau (EPFL, Switzerland) |
| IAC-25 | IAC-25,A3,3A,9,x97728 | "Autonomous AI-Enhanced UAV System for High-Resolution Gravimetric Surveying Along Mars' Equatorial Zone" | Hao Liu, Changfang Zhao, Yifan Wang et al. (Tsinghua University, China) |
| IAC-22 | IAC-22,A3,3A,2,x73207 | "Using UAVs for Future Mission on Mars" | Laura Sopegno, P. Livreri, K.P. Valavanis (Universita di Palermo, Italy) |
| IAC-22 | IAC-22,A3,3A,7,x71026 | "Autonomous Navigation in a GPS Denied Environment. Project MID (Mars Inspection Drone)" | Daniel Betco (Politehnica University of Bucharest, Romania) |
| IAC-22 | IAC-22,A3,3A,9,x72057 | "Design of MARS VTOL Aircraft — A New Hope Towards Martian Search" | Sharvil Joglekar |

---

### Option 2: A3.3B — Mars Exploration: Science, Instruments and Technologies ★

> "Science, instruments and **technologies** for Mars missions including expected experiments."

| 항목 | 내용 |
|------|------|
| **Symposium** | A3 — IAF Space Exploration Symposium |
| **Type** | Oral |
| **Co-Chairs** | Vincenzo Giorgio (Thales Alenia Space, Italy), Pierre W. Bousquet (CNES, France) |
| **장점** | "Technologies"를 명시 — CMG를 화성 비행의 enabling technology로 포지셔닝 가능. 화성 aerial vehicle 논문 선례 있음. Scope 문제 없음 |
| **단점** | A3.3A 대비 화성 회전익기 논문 수가 적음. Reviewer가 센서/계측 중심일 수 있음 |

**이 세션에 제출된 화성 항공기/로봇 논문 선례:**

| 연도 | 논문 코드 | 제목 | 저자 |
|------|----------|------|------|
| IAC-24 | IAC-24,A3,3B,13,x85085 | "Phase-A Design of a Mars Exploration Aerial Vehicle" | Gennaro Barbato, Giuseppe Pezzella, Antonio Viviani (Univ. della Campania, Italy) |
| IAC-24 | IAC-24,A3,3B,16,x90335 | "Enhanced MADDPG with Energy Awareness for Cooperative Path Planning of UAV and UGV on Mars" | Mahya Ramezani, M.A. Amiri Atashgah (Univ. Luxembourg / Univ. Tehran) |

---

### Option 3: C1.1 / C1.2 — Attitude Dynamics

> "Advances in spacecraft attitude dynamics and control, as well as **design, testing and performance of novel attitude sensors and actuators**. Covers dynamics and control of multiple interconnected rigid and flexible bodies, including tethered systems, and in-orbit assembly."

| 항목 | 내용 |
|------|------|
| **Symposium** | C1 — IAF Astrodynamics Symposium |
| **Type** | Oral |
| **C1.1 Co-Chairs** | Prof. Marcello Romano (TUM, Germany), Prof. Bin Meng (Beijing Institute of Control Engineering, China) |
| **C1.2 Co-Chairs** | Dr. Feng Jinglang (University of Strathclyde, UK), **Prof. Bang Hyochoong (KAIST, Korea)** |
| **장점** | "Novel attitude actuators" 명시적 요청 — 논문의 핵심 기여(CMG 제어 아키텍처, MPPI singularity avoidance)와 가장 정확한 기술적 매칭. CMG 전문 reviewer 배정 가능성 높음. KAIST 공동의장 |
| **단점** | **"spacecraft"라는 용어가 전통적으로 궤도 위성/우주선을 의미** — 화성 표면 회전익기는 spacecraft가 아닌 planetary aerial vehicle이므로 scope 밖으로 판단될 리스크 |

**이 세션에 제출된 CMG 관련 논문 선례:**

| 연도 | 논문 코드 | 제목 | 저자 |
|------|----------|------|------|
| IAC-21 | IAC-21,C1,x,x,x63759 | "Unwinding and Singularity Free Satellite Attitude Control Using Double-Gimbal Variable-Speed Control Moment Gyro and Sliding Control" | Gargi Das, Manoranjan Sinha (IIT Kharagpur, India) |
| IAC-22 | — | "Single-Gimbal Control Moment Gyro with Spherical Motor Technology for ADCS" | Johnny K.-K. Liao, Thomas B.-X. Yen, Sam S.-R. Lee, Austin Y.-C. Chang (Tensor Tech, Taiwan) |

> **참고**: C1 세션의 CMG 논문은 전부 **위성** 대상이며, 행성 비행체 대상 CMG 논문은 없음.

---

### Option 4: C1.3–C1.5 — Guidance, Navigation and Control

> "Studies and application related to the guidance, navigation and control of Earth-orbiting and interplanetary spacecraft, including formation flying, rendezvous and docking."

| 항목 | 내용 |
|------|------|
| **Symposium** | C1 — IAF Astrodynamics Symposium |
| **Type** | Oral |
| **C1.3 Co-Chairs** | Prof. Shinichiro Sakai (ISAS/JAXA, Japan), Prof. Steve Ulrich (Carleton University, Canada) |
| **C1.4 Co-Chairs** | Prof. Mai Bando (Kyushu University, Japan), Prof. Hanspeter Schaub (University of Colorado, USA) |
| **C1.5 Co-Chairs** | Dr. Puneet Singla (Penn State, USA), Dr. Bernard Lubke-Ossenbeck (OHB, Germany) |
| **장점** | MPPI 제어기, singularity avoidance 알고리즘 관점 강조 시 적합. C1.4의 Hanspeter Schaub는 VSCMG null motion steering law를 최초 제안한 연구자 (Schaub, Vadali, Junkins 1998) — CMG 분야 권위자 |
| **단점** | C1.1/C1.2와 동일한 "spacecraft" scope 문제. 설명에 "Earth-orbiting and interplanetary"로 더 명시적으로 한정 |

---

### Option 5 (Fallback): Interactive Presentations

| 세션 | 설명 |
|------|------|
| **C1.IP** — Interactive Presentations: IAF Astrodynamics | C1 전체 주제를 포괄하는 interactive (포스터형 디지털) 발표. Co-Chairs: Dr. Vladimir Razoumny (RUDN), Prof. Manoranjan Sinha (IIT Kharagpur), Dr. Feng Jinglang (Strathclyde) |
| **A3.IP** — Interactive Presentations: Space Exploration | A3 전체 주제를 포괄하는 interactive 발표 |

> Oral 세션 불채택 시 fallback 옵션.

---

## 기타 관련 가능 세션 (낮은 적합도)

| 세션 | 설명 | Co-Chairs | 적합도 |
|------|------|-----------|--------|
| **D1.2** — Technologies that Enable Space Systems | "Innovations including sensors, components, software techniques, AI, ML, autonomy" | Jill Prince (NASA), Steven Arnold (JHU APL) | 중하 — CMG를 enabling technology로 프레이밍 시 |
| **D3.2B** — Building Blocks: Technologies | "Technologies for robotic and human operations on Moon, Mars and other destinations" | Dr. Raymond G. Clinton (NASA MSFC) | 중하 — 화성 표면 운용 기술로 프레이밍 시 |
| **A5.2** — Human Exploration of Mars | "Scenarios and infrastructure for human Mars exploration" | — | 낮음 — 유인 미션 지원 인프라로 프레이밍 시만 |
| **B4.8** — Small Spacecraft for Deep-Space Exploration | "Innovative small spacecraft designs for exploration beyond Earth orbit" | Dr. Leon Alkalai (Mandala Space Ventures), Prof. Rene Laufer (Lulea Univ.) | 낮음 — stretch fit |

---

## 핵심 쟁점: "Spacecraft" Scope 문제

C1 세션들의 세션 설명에 사용된 "spacecraft"는 전통적으로 **궤도 위성/우주선**을 지칭한다.
본 논문의 플랫폼은 화성 표면에서 비행하는 **planetary aerial vehicle**로, 엄밀히 spacecraft가 아니다.

- **C1 선택 시 리스크**: Reviewer가 "이건 drone이지 spacecraft가 아니다"로 scope 밖 판정 가능
- **A3 선택 시 리스크**: 제어 알고리즘의 기술적 깊이가 "Mars mission" 프레임에서 충분히 평가받지 못할 수 있음

---

## 과거 유사 논문 제출 패턴

| 주제 | 주요 발표 학회 | IAC 세션 |
|------|-------------|---------|
| Ingenuity / MSH 팀 (Grip, Johnson, Pipenberg 등) | AIAA SciTech, IEEE Aerospace, VFS, NASA TM | **IAC 발표 없음** |
| HIT MarsBird 시리즈 (Quan, Zhu, Tang 등) | Acta Astronautica, Aerospace Science and Technology (저널) | **IAC 발표 없음** |
| 화성 회전익기 개념 (유럽 학계) | — | **A3.3A** (다수), **A3.3B** |
| CMG 자세 제어 (위성) | JGCD, Acta Astronautica | **C1.1/C1.2** |
| CMG-드론 (일본 Ouchi, 한국 Yang/Leeghim) | JSME, MDPI Sensors, IEEE ICCAS | **IAC 발표 없음** |
| 행성 aerial vehicle (Titan Dragonfly 등) | — | **A3.5** (Solar System Exploration) |

**결론**: CMG + 행성 회전익기 조합 논문은 IAC 역사상 전무. 직접적 선례가 없어 세션 선택이 판단의 영역.

---

## 추천안

| 우선순위 | 세션 | 프레이밍 전략 | 비고 |
|---------|------|-------------|------|
| **1순위** | **A3.3B** | CMG를 화성 회전익기의 enabling technology로 제시 | "Technologies for Mars"와 직접 매칭. Scope 안전 |
| **2순위** | **A3.3A** | 현재 초안 기준. CMG 보강 회전익기를 차세대 화성 미션 개념으로 제시 | 화성 회전익기 논문 최다 세션 |
| **3순위** | **C1.2** | Novel attitude actuator + MPPI 제어 아키텍처 강조 | 기술적 매칭 최고, but spacecraft scope 리스크. KAIST 공동의장 |

IAF 제출 시스템에서 **primary/secondary 세션 선택이 가능한 경우**:
- Primary: A3.3B, Secondary: C1.2 (또는 그 반대)

---

## 교수님 확인 요청 사항

1. **A3.3B vs A3.3A vs C1.2** 중 어디로 제출할지
2. C1 세션의 "spacecraft" scope 문제를 감수할 의향이 있는지
3. Primary/Secondary 세션 선택이 가능한 경우 조합 방식
