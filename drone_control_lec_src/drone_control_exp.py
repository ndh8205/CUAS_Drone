#!/usr/bin/env python3
"""
==============================================================================
드론 ArUco 마커 기반 자율 착륙 시스템
==============================================================================

[시스템 개요]
이 프로그램은 카메라로 ArUco 마커를 인식하고, PID 제어를 통해 
드론을 마커 위치로 유도하여 정밀 착륙을 수행합니다.

[주요 구성 요소]
1. ArUco 마커 검출 (OpenCV)
2. PID 제어기 (위치/고도 제어)
3. MAVROS 인터페이스 (드론 통신)
4. 상태 머신 (비행 모드 관리)

[좌표계 설명]
- 카메라 좌표계: 
  * X: 오른쪽 (+), 왼쪽 (-)
  * Y: 아래 (+), 위 (-)
  * Z: 카메라에서 멀어지는 방향 (+)
  
- 드론 Body Frame (NED 기준):
  * X: 전방 (+)
  * Y: 우측 (+)
  * Z: 하방 (+)

[작동 흐름]
1. FCU(Flight Controller Unit) 연결 대기
2. 초기 setpoint 발행 (OFFBOARD 모드 진입 조건)
3. 드론 Arming
4. ArUco 마커 검출 대기
5. 마커 연속 검출 시 OFFBOARD 모드 전환
6. PID 제어로 마커 중심 정렬
7. 정렬 완료 시 자동 착륙
"""

# ==============================================================================
# 라이브러리 임포트
# ==============================================================================

import rospy
"""
rospy: ROS의 Python 클라이언트 라이브러리
- 노드 생성, 토픽 구독/발행, 서비스 호출 등 ROS 기능 제공
- rospy.init_node(): 노드 초기화
- rospy.Subscriber(): 토픽 구독
- rospy.Publisher(): 토픽 발행
- rospy.Rate(): 루프 주기 설정
"""

from mavros_msgs.msg import State
"""
State 메시지: 드론의 현재 상태 정보
- connected: FCU 연결 상태 (bool)
- armed: 모터 암 상태 (bool)
- guided: 가이드 모드 여부 (bool)
- mode: 현재 비행 모드 (string, 예: "OFFBOARD", "AUTO.LAND")
"""

from mavros_msgs.srv import SetMode, CommandBool
"""
SetMode 서비스: 드론 비행 모드 변경
- 요청: custom_mode (string)
- 응답: mode_sent (bool)

CommandBool 서비스: 부울 명령 (주로 Arming에 사용)
- 요청: value (bool)
- 응답: success (bool), result (uint8)
"""

from std_msgs.msg import Float64
"""
Float64 메시지: 단일 64비트 부동소수점 값
- 여기서는 고도(altitude) 데이터 수신에 사용
"""

from geometry_msgs.msg import TwistStamped, PoseStamped
"""
TwistStamped: 타임스탬프가 포함된 속도 메시지
- header: 시간 정보 및 좌표 프레임
- twist.linear: 선형 속도 (x, y, z) [m/s]
- twist.angular: 각속도 (x, y, z) [rad/s]

PoseStamped: 타임스탬프가 포함된 위치/자세 메시지
- header: 시간 정보 및 좌표 프레임
- pose.position: 위치 (x, y, z)
- pose.orientation: 자세 (쿼터니언: x, y, z, w)
"""

from sensor_msgs.msg import CompressedImage, CameraInfo
"""
CompressedImage: 압축된 이미지 메시지 (JPEG, PNG 등)
- format: 압축 형식
- data: 압축된 이미지 바이트 배열

CameraInfo: 카메라 캘리브레이션 정보
- K: 3x3 내부 파라미터 행렬 (Intrinsic Matrix)
- D: 왜곡 계수 (Distortion Coefficients)
- R: 회전 행렬
- P: 투영 행렬
"""

from cv_bridge import CvBridge
"""
CvBridge: ROS 이미지 메시지 ↔ OpenCV 이미지 변환
- imgmsg_to_cv2(): ROS → OpenCV
- cv2_to_imgmsg(): OpenCV → ROS
- compressed_imgmsg_to_cv2(): 압축 ROS → OpenCV
"""

import cv2
import numpy as np
import tf.transformations as transformations
"""
tf.transformations: 좌표 변환 유틸리티
- euler_from_quaternion(): 쿼터니언 → 오일러 각도 변환
- quaternion_from_euler(): 오일러 각도 → 쿼터니언 변환
"""

import time


# ==============================================================================
# PID 제어기 클래스
# ==============================================================================

class PIDController:
    """
    PID (Proportional-Integral-Derivative) 제어기
    
    [PID 제어 이론]
    ┌─────────────────────────────────────────────────────────────┐
    │                                                             │
    │   출력 = Kp × e(t) + Ki × ∫e(t)dt + Kd × de(t)/dt          │
    │                                                             │
    │   여기서 e(t) = setpoint - measurement (오차)               │
    │                                                             │
    └─────────────────────────────────────────────────────────────┘
    
    [각 항의 역할]
    
    1. P (비례항): Kp × error
       ┌────────────────────────────────────────────────────────┐
       │ • 현재 오차에 비례하는 제어 출력                        │
       │ • Kp ↑: 빠른 응답, but 오버슈트/진동 가능               │
       │ • Kp ↓: 느린 응답, 정상상태 오차 발생                   │
       │                                                        │
       │ 예시: 목표 고도 10m, 현재 8m → 오차 2m                  │
       │       Kp=0.5 → P항 출력 = 0.5 × 2 = 1.0 m/s            │
       └────────────────────────────────────────────────────────┘
    
    2. I (적분항): Ki × ∫error dt
       ┌────────────────────────────────────────────────────────┐
       │ • 오차의 누적값에 비례하는 제어 출력                     │
       │ • 정상상태 오차(Steady-State Error) 제거                │
       │ • Ki ↑: 빠른 오차 제거, but 오버슈트/적분 와인드업       │
       │ • Ki ↓: 느린 오차 제거                                  │
       │                                                        │
       │ 주의: Wind-up 현상 - 오차 누적이 과도하게 커지는 문제    │
       │       → Anti-windup 기법 필요 (이 코드에서는 미구현)     │
       └────────────────────────────────────────────────────────┘
    
    3. D (미분항): Kd × d(error)/dt
       ┌────────────────────────────────────────────────────────┐
       │ • 오차의 변화율에 비례하는 제어 출력                     │
       │ • 오버슈트 감소, 시스템 안정화 (댐핑 효과)               │
       │ • Kd ↑: 강한 댐핑, but 노이즈에 민감                    │
       │ • Kd ↓: 약한 댐핑, 진동 가능                            │
       │                                                        │
       │ 주의: 미분은 노이즈를 증폭시킴 → 필터링 권장             │
       └────────────────────────────────────────────────────────┘
    
    [PID 튜닝 가이드라인]
    
    일반적인 튜닝 순서:
    1. Ki = 0, Kd = 0으로 시작
    2. Kp를 증가시키며 적절한 응답 속도 찾기
    3. 진동이 발생하면 Kd를 추가하여 안정화
    4. 정상상태 오차가 있으면 Ki를 소량 추가
    
    Ziegler-Nichols 방법:
    1. Ki = 0, Kd = 0 상태에서 Kp를 증가
    2. 시스템이 지속 진동할 때의 Kp를 Ku (ultimate gain)로 기록
    3. 진동 주기를 Tu로 기록
    4. Kp = 0.6×Ku, Ki = 2×Kp/Tu, Kd = Kp×Tu/8
    """
    
    def __init__(self, Kp, Ki, Kd, setpoint=0.0, output_limits=(None, None)):
        """
        PID 제어기 초기화
        
        Parameters:
        -----------
        Kp : float
            비례 이득 (Proportional Gain)
            - 오차에 대한 즉각적인 반응 강도 결정
            - 단위: [출력 단위] / [오차 단위]
            - 예: 속도 제어 시 [m/s] / [m] = [1/s]
            
        Ki : float
            적분 이득 (Integral Gain)
            - 누적 오차에 대한 반응 강도 결정
            - 단위: [출력 단위] / ([오차 단위] × [시간])
            - 예: [m/s] / [m·s] = [1/s²]
            
        Kd : float
            미분 이득 (Derivative Gain)
            - 오차 변화율에 대한 반응 강도 결정
            - 단위: [출력 단위] × [시간] / [오차 단위]
            - 예: [m/s] × [s] / [m] = [무차원]
            
        setpoint : float
            목표값 (제어하고자 하는 목표)
            - 예: 목표 위치, 목표 고도, 목표 속도 등
            
        output_limits : tuple (lower, upper)
            출력 제한 (Saturation Limits)
            - 물리적 한계나 안전을 위한 출력 클램핑
            - None이면 해당 방향 제한 없음
            - 예: (-0.5, 0.5) → 출력이 ±0.5 m/s로 제한
        
        [드론 제어에서의 적용]
        
        이 코드의 PID 설정:
        ┌──────────┬────────┬────────┬────────┬─────────────────┐
        │ 제어축   │  Kp    │  Ki    │  Kd    │  출력 제한      │
        ├──────────┼────────┼────────┼────────┼─────────────────┤
        │ X (전후) │  0.5   │  0.0   │  0.1   │  ±0.5 m/s      │
        │ Y (좌우) │  0.5   │  0.0   │  0.1   │  ±0.5 m/s      │
        │ Z (고도) │  0.3   │  0.0   │  0.05  │  ±0.3 m/s      │
        └──────────┴────────┴────────┴────────┴─────────────────┘
        
        참고: Ki = 0으로 설정되어 있어 순수 PD 제어기로 동작
              → 정상상태 오차가 발생할 수 있으나 안정성 우선
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint

        # 내부 상태 변수
        self._integral = 0.0      # 적분 누적값: ∫e(t)dt
        self._prev_error = 0.0    # 이전 오차값: e(t-1)
        self._prev_time = None    # 이전 시간: 미분 계산용

        self.output_limits = output_limits  # (하한, 상한)

    def reset(self):
        """
        PID 제어기 상태 초기화
        
        [사용 시점]
        - 새로운 목표로 전환 시
        - 비행 모드 변경 시
        - 오류 복구 후 재시작 시
        
        [초기화 이유]
        적분 와인드업 방지:
        - 이전 오차 누적값이 새로운 제어에 영향을 주는 것 방지
        - 예: 마커 재검출 시 이전 적분값으로 인한 급격한 제어 방지
        """
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = None

    def compute(self, measurement):
        """
        PID 제어 출력 계산
        
        Parameters:
        -----------
        measurement : float
            현재 측정값 (센서로부터 얻은 실제 상태)
            - 예: 현재 위치, 현재 고도, 현재 속도
            
        Returns:
        --------
        output : float
            제어 출력값 (액추에이터에 전달할 명령)
            - 예: 속도 명령, 추력 명령
            
        [계산 과정 상세]
        
        1. 시간 계산
           current_time = time.time()  # 현재 유닉스 시간 [초]
           delta_time = current_time - prev_time  # 시간 간격 Δt
           
        2. 오차 계산
           error = setpoint - measurement
           ┌─────────────────────────────────────────────────┐
           │ 예시: 목표 고도 = 1.0m, 현재 고도 = 1.5m        │
           │       error = 1.0 - 1.5 = -0.5m                 │
           │       → 음수 오차: 현재가 목표보다 높음          │
           │       → 하강 필요 (음의 속도 명령)              │
           └─────────────────────────────────────────────────┘
           
        3. 적분항 계산 (수치 적분 - 사각형 근사)
           integral += error × delta_time
           ┌─────────────────────────────────────────────────┐
           │ ∫e(t)dt ≈ Σ e(k) × Δt                          │
           │                                                 │
           │ 오차│    ┌──┐                                   │
           │     │    │  │  ← error × delta_time             │
           │     │    │  │     (사각형 면적)                  │
           │     └────┴──┴─────→ 시간                        │
           └─────────────────────────────────────────────────┘
           
        4. 미분항 계산 (후진 차분 근사)
           derivative = (error - prev_error) / delta_time
           ┌─────────────────────────────────────────────────┐
           │ de/dt ≈ [e(k) - e(k-1)] / Δt                   │
           │                                                 │
           │ 예시: 이전 오차 = -0.3m, 현재 오차 = -0.5m      │
           │       delta_time = 0.05s (20Hz)                 │
           │       derivative = (-0.5 - (-0.3)) / 0.05       │
           │                  = -0.2 / 0.05 = -4.0 m/s       │
           │       → 오차가 음의 방향으로 증가 중             │
           └─────────────────────────────────────────────────┘
           
        5. PID 출력 합산
           output = Kp × error + Ki × integral + Kd × derivative
           
        6. 출력 제한 (Saturation)
           output = clamp(output, lower_limit, upper_limit)
        """
        current_time = time.time()
        error = self.setpoint - measurement

        # 시간 간격 및 오차 변화량 계산
        # _prev_time이 None이면 첫 실행이므로 delta_time = 0
        delta_time = current_time - self._prev_time if self._prev_time else 0.0
        delta_error = error - self._prev_error if self._prev_time else 0.0

        if delta_time > 0.0:
            # 적분항: 오차 × 시간 누적 (사각형 적분)
            self._integral += error * delta_time
            
            # 미분항: 오차 변화율 (후진 차분)
            derivative = delta_error / delta_time
        else:
            # 첫 실행 시 미분항 = 0 (이전 데이터 없음)
            derivative = 0.0

        # PID 출력 계산
        output = self.Kp * error + self.Ki * self._integral + self.Kd * derivative

        # 출력 제한 적용 (Saturation)
        # 물리적 한계 또는 안전을 위한 클램핑
        lower, upper = self.output_limits
        if lower is not None:
            output = max(lower, output)  # 하한 제한
        if upper is not None:
            output = min(upper, output)  # 상한 제한

        # 다음 계산을 위한 상태 저장
        self._prev_error = error
        self._prev_time = current_time

        return output


# ==============================================================================
# 드론 제어기 클래스
# ==============================================================================

class DroneController: 
    """
    ArUco 마커 기반 드론 자율 착륙 제어기
    
    [시스템 아키텍처]
    
    ┌─────────────────────────────────────────────────────────────────────┐
    │                        DroneController                              │
    ├─────────────────────────────────────────────────────────────────────┤
    │                                                                     │
    │  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐          │
    │  │   카메라      │───▶│  ArUco 검출  │───▶│  오차 계산   │          │
    │  │  Subscriber  │    │   (OpenCV)   │    │ (정규화 오차) │          │
    │  └──────────────┘    └──────────────┘    └──────┬───────┘          │
    │                                                  │                   │
    │  ┌──────────────┐    ┌──────────────┐    ┌──────▼───────┐          │
    │  │   고도/자세   │───▶│  상태 융합   │───▶│ PID 제어기   │          │
    │  │  Subscribers │    │              │    │  (X, Y, Z)   │          │
    │  └──────────────┘    └──────────────┘    └──────┬───────┘          │
    │                                                  │                   │
    │  ┌──────────────┐    ┌──────────────┐    ┌──────▼───────┐          │
    │  │   드론 상태   │───▶│  모드 관리   │◀───│  속도 명령   │          │
    │  │  Subscriber  │    │ (OFFBOARD)   │    │  Publisher   │          │
    │  └──────────────┘    └──────────────┘    └──────────────┘          │
    │                                                                     │
    └─────────────────────────────────────────────────────────────────────┘
    
    [ROS 통신 구조]
    
    Subscribers (토픽 구독):
    ┌─────────────────────────────────────┬────────────────────────────────┐
    │ 토픽                                │ 용도                           │
    ├─────────────────────────────────────┼────────────────────────────────┤
    │ /iris/usb_cam/camera_info           │ 카메라 내부 파라미터           │
    │ /iris/usb_cam/image_raw/compressed  │ 압축 이미지 (ArUco 검출용)     │
    │ /mavros/global_position/rel_alt     │ 상대 고도 (지면 기준)          │
    │ /mavros/state                       │ 드론 상태 (모드, 연결 등)      │
    │ /mavros/local_position/velocity_body│ Body frame 속도               │
    │ /mavros/local_position/pose         │ 위치 및 자세 (쿼터니언)        │
    └─────────────────────────────────────┴────────────────────────────────┘
    
    Publishers (토픽 발행):
    ┌─────────────────────────────────────┬────────────────────────────────┐
    │ 토픽                                │ 용도                           │
    ├─────────────────────────────────────┼────────────────────────────────┤
    │ /mavros/setpoint_velocity/cmd_vel   │ 속도 명령 (Body frame)         │
    └─────────────────────────────────────┴────────────────────────────────┘
    
    Services (서비스 호출):
    ┌─────────────────────────────────────┬────────────────────────────────┐
    │ 서비스                              │ 용도                           │
    ├─────────────────────────────────────┼────────────────────────────────┤
    │ /mavros/cmd/arming                  │ 모터 Arm/Disarm                │
    │ /mavros/set_mode                    │ 비행 모드 변경                 │
    └─────────────────────────────────────┴────────────────────────────────┘
    """
    
    def __init__(self):
        """
        DroneController 초기화
        
        [초기화 순서]
        1. ROS 노드 초기화
        2. 상태 변수 초기화
        3. ArUco 검출기 설정
        4. PID 제어기 생성
        5. Subscriber/Publisher 설정
        """
        
        # ----------------------------------------------------------------------
        # 1. ROS 노드 초기화
        # ----------------------------------------------------------------------
        rospy.init_node("drone_controller")
        """
        ROS 노드 초기화
        - 노드 이름: "drone_controller"
        - ROS 마스터에 노드 등록
        - 이 노드가 다른 노드들과 통신할 수 있게 됨
        
        [ROS 노드 개념]
        노드: ROS에서 실행되는 프로세스의 단위
        - 각 노드는 특정 기능 담당
        - 노드 간 통신: 토픽(Topic), 서비스(Service), 액션(Action)
        """
        
        # ----------------------------------------------------------------------
        # 2. 오차 및 상태 변수 초기화
        # ----------------------------------------------------------------------
        
        # 이미지 기반 오차 (정규화된 값)
        self.norm_x_error = 0.0  # 화면 중앙 기준 좌우 오차 (정규화: -1 ~ 1)
        self.norm_y_error = 0.0  # 화면 중앙 기준 상하 오차 (정규화: -1 ~ 1)
        """
        정규화 오차 (Normalized Error)
        
        [정규화 이유]
        - 해상도 독립적: 640×480이든 1920×1080이든 동일한 제어 파라미터 사용 가능
        - PID 튜닝 용이: 오차 범위가 항상 [-1, 1]로 일정
        
        [계산 방법]
        norm_x_error = pixel_error_x / (image_width / 2)
        norm_y_error = pixel_error_y / (image_height / 2)
        
        ┌────────────────────────────────────────────────────────┐
        │            이미지 좌표계                               │
        │                                                        │
        │   (0,0)───────────────────────────────▶ X (우측 +)     │
        │     │          norm_x = -1  │  norm_x = +1            │
        │     │                       │                          │
        │     │         ┌─────────────┼─────────────┐            │
        │     │         │             │             │            │
        │     │  norm_y │      ───────●───────      │ norm_y     │
        │     │   = -1  │       (중심, 0,0)        │  = +1      │
        │     │         │             │             │            │
        │     │         └─────────────┼─────────────┘            │
        │     │                       │                          │
        │     ▼                                                  │
        │   Y (하단 +)                                           │
        └────────────────────────────────────────────────────────┘
        """
        
        self.z_error = 0.0       # ArUco 마커까지의 거리 [m]
        self.my_alt = 0.0        # 현재 고도 [m]
        
        # OpenCV-ROS 브릿지
        self.bridge = CvBridge()
        """
        CvBridge: ROS 이미지 메시지 ↔ OpenCV 이미지 변환
        - ROS는 sensor_msgs/Image 또는 CompressedImage 사용
        - OpenCV는 numpy 배열 (BGR 형식) 사용
        - CvBridge가 두 형식 간 변환 담당
        """
        
        # 이미지 및 마커 관련 변수
        self.current_image = None     # 현재 프레임 이미지
        self.marker_corners = None    # 검출된 마커의 코너 좌표
        self.marker_ids = None        # 검출된 마커 ID
        
        # 카메라 캘리브레이션 파라미터
        self.camera_matrix = None     # 카메라 내부 파라미터 행렬 (3×3)
        self.dist_coeffs = None       # 렌즈 왜곡 계수
        """
        [카메라 내부 파라미터 행렬 (Intrinsic Matrix)]
        
        K = ┌ fx   0  cx ┐
            │  0  fy  cy │
            └  0   0   1 ┘
            
        fx, fy: 초점 거리 (focal length) [픽셀]
        cx, cy: 주점 (principal point) - 이미지 중심 [픽셀]
        
        [3D → 2D 투영]
        ┌ u ┐       ┌ fx   0  cx ┐   ┌ X/Z ┐
        │ v │ = K × │  0  fy  cy │ × │ Y/Z │
        └ 1 ┘       └  0   0   1 ┘   └  1  ┘
        
        u, v: 이미지 좌표 [픽셀]
        X, Y, Z: 카메라 좌표계에서의 3D 점 [m]
        
        [렌즈 왜곡 계수 (Distortion Coefficients)]
        D = [k1, k2, p1, p2, k3]
        
        k1, k2, k3: 방사 왜곡 계수 (Radial Distortion)
        p1, p2: 접선 왜곡 계수 (Tangential Distortion)
        
        왜곡 모델:
        x_distorted = x(1 + k1*r² + k2*r⁴ + k3*r⁶) + 2*p1*x*y + p2*(r² + 2*x²)
        y_distorted = y(1 + k1*r² + k2*r⁴ + k3*r⁶) + p1*(r² + 2*y²) + 2*p2*x*y
        """
        
        self.marker_size = 0.8  # 마커 실제 크기 [m]
        """
        마커 크기 (Marker Size)
        - ArUco 마커의 한 변 길이 (정사각형)
        - Pose Estimation에 필수: 크기를 알아야 거리 계산 가능
        - 실제 마커와 일치해야 정확한 거리 측정 가능
        
        [거리 계산 원리 - 핀홀 카메라 모델]
        
        실제 크기(S) × 초점 거리(f)
        거리(Z) = ─────────────────────
                    이미지 크기(s)
        
        예시: 0.8m 마커가 이미지에서 100픽셀로 보임
              focal_length = 500 픽셀
              Z = (0.8 × 500) / 100 = 4.0m
        """
        
        self.distance = 0.0           # 마커까지의 유클리드 거리
        self.marker_detected = False  # 마커 검출 플래그

        # 현재 속도 변수 (Body Frame)
        self.current_vel_x = 0.0  # 전방 속도 [m/s]
        self.current_vel_y = 0.0  # 우측 속도 [m/s]
        self.current_vel_z = 0.0  # 하강 속도 [m/s]
        """
        Body Frame 속도
        
        ┌──────────────────────────────────────────────────────┐
        │              드론 Body Frame (NED)                   │
        │                                                      │
        │                    +X (전방)                         │
        │                      ↑                               │
        │                      │                               │
        │         +Y (우측) ←──┼──→                            │
        │                      │                               │
        │                      ↓                               │
        │                    +Z (하방)                         │
        │                                                      │
        │   NED: North-East-Down 좌표계를 드론에 적용          │
        └──────────────────────────────────────────────────────┘
        """

        # 드론 자세 (오일러 각도)
        self.roll = 0.0   # 롤 각도 [rad] - X축 회전
        self.pitch = 0.0  # 피치 각도 [rad] - Y축 회전
        self.yaw = 0.0    # 요 각도 [rad] - Z축 회전
        """
        오일러 각도 (Euler Angles)
        
        ┌──────────────────────────────────────────────────────┐
        │                                                      │
        │   Roll (φ): X축 회전 - 좌우 기울기                    │
        │   ┌─────┐         ┌─────┐                            │
        │   │  ○  │   →     │  ◐  │   (+Roll: 우측 기울임)      │
        │   └─────┘         └─────┘                            │
        │                                                      │
        │   Pitch (θ): Y축 회전 - 전후 기울기                   │
        │   ══════         ══╲══                               │
        │     ↓              ↓  (+Pitch: 기수 하강)            │
        │                                                      │
        │   Yaw (ψ): Z축 회전 - 좌우 회전                       │
        │     ↑              ↗                                 │
        │   ══════         ══════  (+Yaw: 시계 반대 방향)       │
        │                                                      │
        └──────────────────────────────────────────────────────┘
        
        쿼터니언 → 오일러 변환 시 gimbal lock 주의:
        - Pitch = ±90° 근처에서 Roll과 Yaw 구분 불가
        - 드론의 일반적 비행에서는 문제 없음
        """

        # ----------------------------------------------------------------------
        # 3. ArUco 검출기 설정
        # ----------------------------------------------------------------------
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
        """
        ArUco 딕셔너리 선택
        
        [ArUco 마커란?]
        - 흑백 정사각형 패턴의 시각적 마커
        - 고유 ID를 가지며, 자세(Pose) 추정 가능
        - QR 코드보다 빠른 검출, 3D 자세 추정 특화
        
        [딕셔너리 종류]
        ┌────────────────────┬────────────┬──────────────────────┐
        │ 딕셔너리           │ 마커 수    │ 특징                 │
        ├────────────────────┼────────────┼──────────────────────┤
        │ DICT_4X4_50        │ 50개       │ 4×4 비트, 빠른 검출  │
        │ DICT_4X4_100       │ 100개      │ 4×4 비트             │
        │ DICT_4X4_250       │ 250개      │ 4×4 비트             │
        │ DICT_4X4_1000      │ 1000개     │ 4×4 비트, 많은 ID    │
        │ DICT_5X5_50~1000   │ 다양       │ 5×5 비트, 더 robust  │
        │ DICT_6X6_50~1000   │ 다양       │ 6×6 비트             │
        │ DICT_7X7_50~1000   │ 다양       │ 7×7 비트, 고해상도   │
        │ DICT_ARUCO_ORIGINAL│ 1024개     │ 원본 ArUco           │
        └────────────────────┴────────────┴──────────────────────┘
        
        DICT_4X4_1000 선택 이유:
        - 4×4: 작은 비트 수 → 빠른 검출, 원거리 인식 유리
        - 1000개: 충분한 마커 ID 제공
        - 드론 착륙 마커로 적합
        """
        
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        """
        ArUco 검출 파라미터
        
        주요 파라미터 (기본값 사용 중, 필요시 조정 가능):
        
        [적응적 이진화 관련]
        - adaptiveThreshWinSizeMin: 최소 윈도우 크기 (기본: 3)
        - adaptiveThreshWinSizeMax: 최대 윈도우 크기 (기본: 23)
        - adaptiveThreshWinSizeStep: 윈도우 증가 단계 (기본: 10)
        - adaptiveThreshConstant: 상수 C (기본: 7)
        
        [코너 정제 관련]
        - cornerRefinementMethod: 코너 정제 방법
          * CORNER_REFINE_NONE: 정제 안 함 (빠름)
          * CORNER_REFINE_SUBPIX: 서브픽셀 정제 (정확)
          * CORNER_REFINE_CONTOUR: 윤곽선 기반 정제
          * CORNER_REFINE_APRILTAG: AprilTag 방식
        
        [후보 필터링]
        - minMarkerPerimeterRate: 최소 마커 둘레 비율
        - maxMarkerPerimeterRate: 최대 마커 둘레 비율
        - polygonalApproxAccuracyRate: 다각형 근사 정확도
        
        [오검출 필터링]
        - minCornerDistanceRate: 코너 간 최소 거리 비율
        - minDistanceToBorder: 테두리까지 최소 거리
        - minMarkerDistanceRate: 마커 간 최소 거리 비율
        
        튜닝 예시 (원거리 검출 향상):
        self.aruco_params.adaptiveThreshWinSizeMin = 3
        self.aruco_params.adaptiveThreshWinSizeMax = 30
        self.aruco_params.minMarkerPerimeterRate = 0.01
        """

        # ----------------------------------------------------------------------
        # 4. PID 제어기 설정
        # ----------------------------------------------------------------------
        self.pid_x = PIDController(Kp=135.0, Ki=240.0, Kd=0.0, output_limits=(-0.5, 0.5))
        self.pid_y = PIDController(Kp=321.0, Ki=-151.0, Kd=0.0, output_limits=(-0.5, 0.5))
        self.pid_z = PIDController(Kp=125.0, Ki=-91.0, Kd=0.05, output_limits=(-0.3, 0.3))
        """
        PID 제어기 인스턴스 생성
        
        [제어 축별 PID 설정]
        
        X축 (전후 방향) - pid_x:
        - Kp=0.5: 중간 정도의 비례 이득
        - Ki=0.0: 적분 없음 (정상상태 오차 허용)
        - Kd=0.1: 약한 미분 (오버슈트 방지)
        - 출력 제한: ±0.5 m/s
        
        Y축 (좌우 방향) - pid_y:
        - X축과 동일한 설정 (대칭적 제어)
        
        Z축 (고도 방향) - pid_z:
        - Kp=0.3: X,Y보다 낮은 이득 (부드러운 고도 변화)
        - Ki=0.0: 적분 없음
        - Kd=0.05: 약한 미분
        - 출력 제한: ±0.3 m/s (안전한 수직 속도)
        
        [튜닝 시 고려사항]
        1. 드론 질량 및 추력 특성
        2. 카메라 프레임레이트 (지연)
        3. 바람 등 외란 (Disturbance)
        4. 안전 마진
        """

        # ----------------------------------------------------------------------
        # 5. ROS Subscriber 설정
        # ----------------------------------------------------------------------
        
        self.camera_info_sub = rospy.Subscriber(
            '/iris/usb_cam/camera_info',  # 토픽 이름
            CameraInfo,                    # 메시지 타입
            self.camera_info_callback      # 콜백 함수
        )
        """
        카메라 정보 구독자
        - 카메라 캘리브레이션 파라미터 수신
        - 보통 한 번만 수신하면 충분 (정적 정보)
        """
        
        self.altitude_sub = rospy.Subscriber(
            "/mavros/global_position/rel_alt",
            Float64,
            self.altitude_callback
        )
        """
        고도 구독자
        - rel_alt: 이륙 지점 기준 상대 고도
        - 착륙 판단 및 고도 제어에 사용
        """
        
        self.state_sub = rospy.Subscriber(
            '/mavros/state',
            State,
            self.state_callback
        )
        """
        드론 상태 구독자
        - 연결 상태, ARM 상태, 비행 모드 확인
        - OFFBOARD 모드 전환 조건 판단에 필수
        """
        
        self.image_sub = rospy.Subscriber(
            '/iris/usb_cam/image_raw/compressed',
            CompressedImage,
            self.image_callback
        )
        """
        이미지 구독자
        - 압축 이미지 수신 (JPEG)
        - 대역폭 절약을 위해 압축 이미지 사용
        - ArUco 마커 검출의 입력
        """
        
        self.velocity_sub = rospy.Subscriber(
            '/mavros/local_position/velocity_body',
            TwistStamped,
            self.velocity_callback
        )
        """
        속도 구독자
        - Body frame 기준 현재 속도
        - 디버깅 및 모니터링용
        """
        
        self.attitude_sub = rospy.Subscriber(
            '/mavros/local_position/pose',
            PoseStamped,
            self.attitude_callback
        )
        """
        자세 구독자
        - 쿼터니언으로 자세 수신
        - 오일러 각도로 변환하여 표시
        """
        
        self.state = State()  # 상태 메시지 초기화

        # ----------------------------------------------------------------------
        # 6. ROS Publisher 설정
        # ----------------------------------------------------------------------
        
        self.cmd_pub = rospy.Publisher(
            '/mavros/setpoint_velocity/cmd_vel',  # 토픽 이름
            TwistStamped,                         # 메시지 타입
            queue_size=10                         # 큐 크기
        )
        """
        속도 명령 발행자
        
        [토픽 설명]
        /mavros/setpoint_velocity/cmd_vel:
        - OFFBOARD 모드에서 드론에 속도 명령 전달
        - Body frame 또는 World frame 속도
        - TwistStamped: 타임스탬프 포함 속도 메시지
        
        [큐 크기 (queue_size)]
        - 발행 속도가 구독 속도보다 빠를 때 버퍼링
        - queue_size=10: 최대 10개 메시지 버퍼링
        - 오래된 메시지는 드롭됨 (최신 정보 우선)
        """
        
        self.rate = rospy.Rate(20)  # 20Hz (50ms 주기)
        """
        제어 루프 주기 설정
        
        [20Hz 선택 이유]
        - PX4 OFFBOARD 모드 요구사항: 최소 2Hz 이상
        - 권장: 10~50Hz
        - 20Hz: 50ms 간격으로 명령 발행
        - 너무 높으면 컴퓨팅 부하 증가
        - 너무 낮으면 제어 성능 저하
        """

        # ----------------------------------------------------------------------
        # 7. 제어 관련 카운터 및 플래그
        # ----------------------------------------------------------------------
        
        self.marker_detected_count = 0    # 연속 마커 검출 횟수
        self.required_detection_count = 50  # OFFBOARD 전환 필요 횟수
        """
        마커 검출 카운터
        
        [안전을 위한 연속 검출 요구]
        - 단발성 오검출 방지
        - 50회 연속 검출 시 OFFBOARD 모드 전환
        - 20Hz에서 50회 = 2.5초 연속 검출 필요
        
        [동작 흐름]
        1. 마커 검출됨 → marker_detected_count++
        2. 마커 미검출 → marker_detected_count = 0
        3. count >= 50 → OFFBOARD 모드 전환 및 추적 시작
        """

    # ==========================================================================
    # 콜백 함수들 (Callback Functions)
    # ==========================================================================

    def camera_info_callback(self, msg):
        """
        카메라 정보 콜백
        
        [호출 시점]
        /iris/usb_cam/camera_info 토픽에 새 메시지 도착 시
        
        [수신 데이터]
        - msg.K: 3×3 카메라 내부 파라미터 행렬 (9개 원소의 1D 배열)
        - msg.D: 왜곡 계수 (보통 5개 원소)
        
        Parameters:
        -----------
        msg : CameraInfo
            카메라 캘리브레이션 정보
        """
        # K 행렬: 1D 배열 → 3×3 행렬로 reshape
        self.camera_matrix = np.array(msg.K).reshape(3, 3)
        
        # 왜곡 계수
        self.dist_coeffs = np.array(msg.D)
        
        """
        [camera_matrix 구조]
        
        ┌ K[0]  K[1]  K[2] ┐   ┌ fx   0   cx ┐
        │ K[3]  K[4]  K[5] │ = │  0  fy   cy │
        └ K[6]  K[7]  K[8] ┘   └  0   0    1 ┘
        
        실제 값 예시 (640×480 카메라):
        fx ≈ 500, fy ≈ 500, cx ≈ 320, cy ≈ 240
        """

    def velocity_callback(self, msg):
        """
        속도 콜백
        
        Body frame 기준 현재 속도 업데이트
        
        Parameters:
        -----------
        msg : TwistStamped
            타임스탬프 포함 속도 메시지
            - msg.twist.linear: 선형 속도 (x, y, z)
            - msg.twist.angular: 각속도 (x, y, z)
        """
        self.current_vel_x = msg.twist.linear.x  # 전방 속도
        self.current_vel_y = msg.twist.linear.y  # 우측 속도
        self.current_vel_z = msg.twist.linear.z  # 하강 속도

    def attitude_callback(self, msg):
        """
        자세 콜백
        
        쿼터니언으로 수신된 자세를 오일러 각도로 변환
        
        Parameters:
        -----------
        msg : PoseStamped
            위치 및 자세 메시지
            - msg.pose.position: 위치 (x, y, z)
            - msg.pose.orientation: 자세 (쿼터니언)
        
        [쿼터니언 (Quaternion)]
        q = w + xi + yj + zk
        
        장점:
        - Gimbal lock 없음
        - 보간(Slerp) 용이
        - 수치 안정성
        
        단점:
        - 직관적 이해 어려움 → 오일러 각도로 변환하여 표시
        """
        orientation_q = msg.pose.orientation
        orientation_list = [
            orientation_q.x, 
            orientation_q.y, 
            orientation_q.z, 
            orientation_q.w
        ]
        
        # 쿼터니언 → 오일러 각도 변환
        # 반환 순서: (roll, pitch, yaw)
        (self.roll, self.pitch, self.yaw) = transformations.euler_from_quaternion(
            orientation_list
        )

    def image_callback(self, msg):
        """
        이미지 콜백 - 핵심 마커 검출 및 오차 계산
        
        [처리 흐름]
        1. 압축 이미지 → OpenCV 이미지 변환
        2. ArUco 마커 검출
        3. 마커 Pose 추정
        4. 정규화 오차 계산
        5. 시각화
        
        Parameters:
        -----------
        msg : CompressedImage
            JPEG 압축된 이미지 메시지
        """
        try:
            # ------------------------------------------------------------------
            # 1단계: 압축 이미지 → OpenCV 이미지 변환
            # ------------------------------------------------------------------
            
            # 바이트 배열 → numpy 배열
            np_arr = np.frombuffer(msg.data, np.uint8)
            
            # JPEG 디코딩
            self.current_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            """
            cv2.imdecode():
            - 압축된 이미지 바이트 → OpenCV 이미지 (numpy 배열)
            - cv2.IMREAD_COLOR: BGR 3채널 컬러 이미지로 디코딩
            - 반환: height × width × 3 numpy 배열
            """
            
            # 카메라 파라미터가 있어야 Pose 추정 가능
            if self.camera_matrix is not None:
                
                # ----------------------------------------------------------
                # 2단계: 이미지 중심점 계산
                # ----------------------------------------------------------
                h, w = self.current_image.shape[:2]  # 높이, 너비
                center_x, center_y = w//2, h//2       # 이미지 중심 좌표
                """
                이미지 중심점
                - 마커가 화면 중앙에 오도록 제어하기 위한 기준점
                - 640×480 이미지: center = (320, 240)
                """

                # ----------------------------------------------------------
                # 3단계: ArUco 마커 검출
                # ----------------------------------------------------------
                corners, ids, rejected = cv2.aruco.detectMarkers(
                    self.current_image,     # 입력 이미지
                    self.aruco_dict,        # ArUco 딕셔너리
                    parameters=self.aruco_params  # 검출 파라미터
                )
                """
                cv2.aruco.detectMarkers() 반환값:
                
                corners: 검출된 마커들의 코너 좌표
                - 형태: list of numpy arrays
                - 각 마커: shape (1, 4, 2) - 4개 코너의 (x, y) 좌표
                - 코너 순서: 좌상단 → 시계방향
                
                  ┌──────────────────┐
                  │ corners[0][0][0] │───▶ (x0, y0) 좌상단
                  │ corners[0][0][1] │───▶ (x1, y1) 우상단
                  │ corners[0][0][2] │───▶ (x2, y2) 우하단
                  │ corners[0][0][3] │───▶ (x3, y3) 좌하단
                  └──────────────────┘
                
                ids: 검출된 마커들의 ID
                - 형태: numpy array, shape (N, 1)
                - N: 검출된 마커 수
                
                rejected: 마커 후보였으나 검증 실패한 영역
                - 디버깅용
                """
                
                # ----------------------------------------------------------
                # 4단계: 마커가 검출된 경우 - Pose 추정 및 오차 계산
                # ----------------------------------------------------------
                if ids is not None and len(ids) > 0:
                    
                    # Pose 추정 (회전 벡터, 변환 벡터)
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                        corners,              # 마커 코너 좌표
                        self.marker_size,     # 마커 실제 크기 [m]
                        self.camera_matrix,   # 카메라 내부 파라미터
                        self.dist_coeffs      # 렌즈 왜곡 계수
                    )
                    """
                    estimatePoseSingleMarkers() 반환값:
                    
                    rvecs: 회전 벡터 (Rotation Vectors)
                    - 형태: (N, 1, 3) - 각 마커의 (rx, ry, rz)
                    - Rodrigues 회전 표현: 방향 = 회전축, 크기 = 회전각 [rad]
                    - 카메라 좌표계에서 마커 좌표계로의 회전
                    
                    tvecs: 변환 벡터 (Translation Vectors)
                    - 형태: (N, 1, 3) - 각 마커의 (tx, ty, tz)
                    - 카메라 원점에서 마커 중심까지의 3D 벡터 [m]
                    - tvecs[0][0] = [X, Y, Z]
                      * X: 카메라 기준 우측 (+)
                      * Y: 카메라 기준 하단 (+)
                      * Z: 카메라 기준 전방 (+) = 거리
                    
                    [좌표계 시각화]
                    
                           카메라 렌즈 방향
                              ↓
                    ┌─────────────────────────┐
                    │       카메라            │
                    │   Y ↓    │              │
                    │    ─────●───▶ X         │
                    │         │               │
                    │         ▼ Z (전방)      │
                    └─────────────────────────┘
                                │
                                │  거리 = tvecs[0][0][2]
                                ▼
                         ┌───────────┐
                         │  ArUco    │
                         │  마커     │
                         └───────────┘
                    """
                    
                    # 마커의 화면상 중심점 계산
                    # 4개 코너의 평균 → 마커 중심 픽셀 좌표
                    marker_center = np.mean(corners[0][0], axis=0)
                    """
                    corners[0][0]: 첫 번째 마커의 4개 코너 좌표
                    - shape: (4, 2)
                    - np.mean(..., axis=0): 4개 점의 평균 → 중심점
                    - marker_center: [cx, cy] 픽셀 좌표
                    """
                    
                    # 화면 중심으로부터의 픽셀 오차 계산
                    pixel_error_x = marker_center[0] - center_x  # 양수: 마커가 오른쪽
                    pixel_error_y = marker_center[1] - center_y  # 양수: 마커가 아래
                    """
                    픽셀 오차 계산
                    
                    ┌────────────────────────────────────────────┐
                    │                이미지                      │
                    │                                            │
                    │     마커 중심●─────────────┐               │
                    │              │             │               │
                    │              │pixel_error_y│               │
                    │              │             │               │
                    │              └─────────────● 이미지 중심   │
                    │                pixel_error_x               │
                    │                                            │
                    └────────────────────────────────────────────┘
                    
                    마커가 화면 우하단에 있으면:
                    - pixel_error_x > 0 (오른쪽)
                    - pixel_error_y > 0 (아래)
                    """
                    
                    # 픽셀 오차를 정규화 (-1 ~ 1 범위)
                    self.norm_x_error = pixel_error_x / (w/2)  # 우측이 양수
                    self.norm_y_error = pixel_error_y / (h/2)  # 아래가 양수
                    """
                    정규화 오차
                    
                    범위: [-1, 1]
                    - -1: 마커가 이미지 왼쪽/위쪽 끝
                    -  0: 마커가 이미지 중앙
                    - +1: 마커가 이미지 오른쪽/아래쪽 끝
                    
                    예시 (640×480 이미지):
                    - 마커가 (480, 360)에 있으면:
                    - pixel_error_x = 480 - 320 = 160
                    - pixel_error_y = 360 - 240 = 120
                    - norm_x_error = 160 / 320 = 0.5
                    - norm_y_error = 120 / 240 = 0.5
                    """
                    
                    # Z축 거리 (마커까지의 전방 거리)
                    self.z_error = tvecs[0][0][2]  # [m]
                    
                    # 상태 플래그 업데이트
                    self.marker_detected = True
                    
                    # 유클리드 거리 (디버깅/표시용)
                    self.distance = np.linalg.norm(tvecs[0][0])
                    """
                    거리 계산
                    
                    유클리드 거리 = √(X² + Y² + Z²)
                    - 마커까지의 직선 거리
                    - Z (전방 거리)와 약간 다름 (X, Y 오프셋 포함)
                    """

                    # 시각화: 마커 및 축 그리기
                    cv2.aruco.drawDetectedMarkers(
                        self.current_image, corners, ids
                    )
                    cv2.aruco.drawAxis(
                        self.current_image,
                        self.camera_matrix,
                        self.dist_coeffs,
                        rvecs[0], tvecs[0], 0.1  # 축 길이 0.1m
                    )
                    """
                    drawAxis(): 마커의 좌표축 시각화
                    - 빨강: X축
                    - 초록: Y축
                    - 파랑: Z축
                    - 축 길이: 0.1m (실제 스케일 반영)
                    """
                    
                else:
                    # ----------------------------------------------------------
                    # 마커 미검출 시: 오차 초기화
                    # ----------------------------------------------------------
                    self.norm_x_error = 0.0
                    self.norm_y_error = 0.0
                    self.z_error = 0.0
                    self.marker_detected = False

                # 마커 정보 저장 (시각화용)
                self.marker_corners = corners
                self.marker_ids = ids
            
            # 이미지 화면 표시
            self.display_image()
            
        except Exception as e:
            rospy.logerr(f"Error in image processing: {e}")
            """
            예외 처리
            - 이미지 디코딩 실패
            - ArUco 검출 오류
            - numpy 연산 오류
            → 로그 기록 후 계속 실행 (드론 안전)
            """

    def display_image(self):
        """
        이미지 시각화 (디버깅/모니터링용)
        
        [표시 내용]
        1. ArUco 마커 윤곽선 및 ID
        2. 마커 중심점 및 연결선
        3. 이미지 중심 십자선
        4. 드론 상태 정보 (고도, 자세)
        """
        if self.current_image is not None:
            display_img = self.current_image.copy()  # 원본 보존
            h, w = display_img.shape[:2]
            center_x, center_y = w//2, h//2
            
            # 마커가 검출된 경우
            if self.marker_corners is not None and len(self.marker_corners) > 0:
                # 마커 윤곽선 및 ID 표시
                cv2.aruco.drawDetectedMarkers(
                    display_img, self.marker_corners, self.marker_ids
                )
                
                # 각 마커에 대해
                for corner in self.marker_corners:
                    # 마커 중심점 계산 및 표시
                    marker_center = np.mean(corner[0], axis=0).astype(int)
                    cv2.circle(display_img, tuple(marker_center), 5, (0,0,255), -1)
                    """
                    cv2.circle() 파라미터:
                    - 이미지, 중심점, 반지름, 색상(BGR), 두께(-1: 채움)
                    - (0,0,255): 빨간색
                    """
                    
                    # 이미지 중심 → 마커 중심 연결선
                    cv2.line(
                        display_img,
                        (center_x, center_y),   # 시작점
                        tuple(marker_center),    # 끝점
                        (255,0,0),               # 파란색
                        2                         # 두께
                    )

            # 이미지 중심 십자선 표시
            cv2.line(display_img, (center_x-20, center_y), (center_x+20, center_y), 
                    (0,255,0), 2)  # 수평선 (초록)
            cv2.line(display_img, (center_x, center_y-20), (center_x, center_y+20), 
                    (0,255,0), 2)  # 수직선 (초록)
            
            # 상태 정보 텍스트
            info_text = [
                f"Altitude: {self.my_alt:.2f}m",
                f"Roll: {np.degrees(self.roll):.2f}deg",
                f"Pitch: {np.degrees(self.pitch):.2f}deg",
                f"Yaw: {np.degrees(self.yaw):.2f}deg"
            ]
            """
            np.degrees(): 라디안 → 도(degree) 변환
            - 오일러 각도는 라디안으로 저장됨
            - 사람이 읽기 쉽게 도 단위로 변환
            """

            # 텍스트 표시
            for i, text in enumerate(info_text):
                cv2.putText(
                    display_img, 
                    text, 
                    (10, 20 + i*20),  # 위치 (20픽셀 간격)
                    cv2.FONT_HERSHEY_SIMPLEX, 
                    0.5,              # 폰트 크기
                    (0,255,0),        # 초록색
                    1                  # 두께
                )
            
            # 마커 ID 표시
            if self.marker_ids is not None and len(self.marker_ids) > 0:
                cv2.putText(
                    display_img, 
                    f"Marker ID: {self.marker_ids[0]}", 
                    (10, 160),
                    cv2.FONT_HERSHEY_SIMPLEX, 
                    0.5, (0,255,0), 1
                )
            
            # OpenCV 윈도우에 표시
            cv2.imshow("Drone View", display_img)
            cv2.waitKey(1)
            """
            cv2.waitKey(1):
            - 1ms 동안 키 입력 대기
            - GUI 이벤트 처리에 필수
            - 0이면 무한 대기
            """

    def altitude_callback(self, msg):
        """
        고도 콜백
        
        Parameters:
        -----------
        msg : Float64
            상대 고도 [m] (이륙 지점 기준)
        """
        self.my_alt = msg.data

    def state_callback(self, msg):
        """
        드론 상태 콜백
        
        Parameters:
        -----------
        msg : State
            드론 상태 정보
            - connected: FCU 연결 상태
            - armed: 모터 암 상태
            - mode: 현재 비행 모드
        """
        self.state = msg

    # ==========================================================================
    # 드론 제어 서비스 함수들
    # ==========================================================================

    def arm(self):
        """
        드론 Arming (모터 활성화)
        
        [Arming이란?]
        - 모터에 전원을 공급하여 회전 가능 상태로 만듦
        - 안전을 위해 명시적 Arming 필요
        - Disarm: 모터 비활성화
        
        [Arming 조건 (일반적)]
        - RC 송신기 연결 또는 GCS 연결
        - GPS lock (위치 모드 필요 시)
        - 배터리 충분
        - 센서 캘리브레이션 완료
        - Pre-arm 체크 통과
        """
        rospy.wait_for_service('/mavros/cmd/arming')
        """
        wait_for_service(): 서비스가 사용 가능해질 때까지 대기
        - ROS 서비스: 요청-응답 패턴
        - 토픽과 달리 동기적 통신
        """
        
        try:
            # 서비스 프록시 생성
            arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            """
            ServiceProxy: 서비스 호출을 위한 클라이언트 객체
            - 첫 번째 인자: 서비스 이름
            - 두 번째 인자: 서비스 타입
            """
            
            # Arming 요청 (True = Arm, False = Disarm)
            response = arm_service(True)
            
            if response.success:
                rospy.loginfo("Vehicle armed")
            else:
                rospy.logerr("Failed to arm vehicle")
                
        except rospy.ServiceException as e:
            rospy.logerr("Arming failed: %s", str(e))

    def set_mav_frame(self, frame_id):
        """
        MAV Frame 설정 (미사용)
        
        [MAV Frame 종류]
        - MAV_FRAME_LOCAL_NED (1): 로컬 NED 프레임
        - MAV_FRAME_BODY_NED (8): Body NED 프레임
        - MAV_FRAME_LOCAL_OFFSET_NED (7): 로컬 오프셋 NED
        
        참고: 이 함수는 현재 코드에서 사용되지 않음
        """
        rospy.wait_for_service('/mavros/setpoint_velocity/mav_frame')
        try:
            mav_frame_service = rospy.ServiceProxy(
                '/mavros/setpoint_velocity/mav_frame', SetMode
            )
            response = mav_frame_service(frame_id)
            rospy.loginfo("MAV frame set successfully to %d", frame_id)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

    def set_mode(self, mode):
        """
        비행 모드 설정
        
        Parameters:
        -----------
        mode : str
            설정할 비행 모드
            - "OFFBOARD": 외부 컴퓨터 제어 모드
            - "AUTO.LAND": 자동 착륙 모드
            - "MANUAL": 수동 조종 모드
            - "STABILIZED": 자세 안정화 모드
            - "POSITION": GPS 위치 유지 모드
        
        [OFFBOARD 모드 특징]
        - 컴패니언 컴퓨터 (Raspberry Pi 등)에서 명령 수신
        - mavros를 통해 setpoint 토픽으로 명령 전달
        - 안전 조건: 모드 전환 전 최소 100개의 setpoint 발행 필요
        - 2초 이상 setpoint 미수신 시 failsafe 동작
        
        [AUTO.LAND 모드]
        - 현재 위치에서 수직 하강
        - 지면 감지 시 모터 정지
        - GPS 불필요 (고도계만 사용)
        """
        rospy.wait_for_service('/mavros/set_mode')
        try:
            set_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            response = set_mode_service(0, mode)
            """
            SetMode 서비스 인자:
            - base_mode (uint8): MAV_MODE 플래그 (보통 0 사용)
            - custom_mode (string): PX4 커스텀 모드 문자열
            """
            
            if response.mode_sent:
                rospy.loginfo(f"Set mode to {mode}")
            else:
                rospy.logerr(f"Failed to set mode to {mode}")
        except rospy.ServiceException as e:
            rospy.logerr("Set mode failed: %s", str(e))

    def offboard_mode(self):
        """
        OFFBOARD 모드 전환
        
        현재 모드가 OFFBOARD가 아닌 경우에만 전환 시도
        """
        if self.state.mode != "OFFBOARD":
            self.set_mode("OFFBOARD")
            rospy.loginfo("Offboard mode enabled.")

    def land(self):
        """
        자동 착륙 실행
        
        AUTO.LAND 모드로 전환하여 착륙 수행
        """
        rospy.loginfo("Landing...")
        self.set_mode("AUTO.LAND")
        rospy.sleep(5)  # 5초 대기 (착륙 완료 대기)

    # ==========================================================================
    # 메인 제어 루프
    # ==========================================================================

    def control_drone(self):
        """
        메인 드론 제어 루프
        
        [전체 동작 흐름]
        
        ┌─────────────────────────────────────────────────────────────────┐
        │                    control_drone() 흐름도                       │
        ├─────────────────────────────────────────────────────────────────┤
        │                                                                 │
        │   ┌─────────────────┐                                          │
        │   │  FCU 연결 대기  │                                          │
        │   └────────┬────────┘                                          │
        │            ▼                                                    │
        │   ┌─────────────────┐                                          │
        │   │ 초기 Setpoint   │  ← OFFBOARD 전환 조건 충족              │
        │   │   100회 발행    │    (PX4는 setpoint 스트림 필요)          │
        │   └────────┬────────┘                                          │
        │            ▼                                                    │
        │   ┌─────────────────┐                                          │
        │   │    드론 Arm     │                                          │
        │   └────────┬────────┘                                          │
        │            ▼                                                    │
        │   ┌─────────────────────────────────────────────┐              │
        │   │              메인 제어 루프                  │              │
        │   │  ┌───────────────────────────────────────┐ │              │
        │   │  │      마커 검출됨?                     │ │              │
        │   │  └──────────┬──────────────┬─────────────┘ │              │
        │   │         Yes │              │ No            │              │
        │   │             ▼              ▼               │              │
        │   │  ┌───────────────┐  ┌───────────────────┐ │              │
        │   │  │ 연속 검출 50회│  │ 카운터 리셋       │ │              │
        │   │  │ 이상?        │  │ 호버링 유지       │ │              │
        │   │  └───────┬──────┘  └───────────────────┘ │              │
        │   │      Yes │                                │              │
        │   │          ▼                                │              │
        │   │  ┌───────────────┐                       │              │
        │   │  │ OFFBOARD 전환 │                       │              │
        │   │  │ PID 제어 실행 │                       │              │
        │   │  └───────┬───────┘                       │              │
        │   │          ▼                                │              │
        │   │  ┌───────────────┐                       │              │
        │   │  │ 정렬 완료?    │                       │              │
        │   │  │ X,Y < 0.05    │                       │              │
        │   │  │ Z < 0.1m      │                       │              │
        │   │  └───────┬───────┘                       │              │
        │   │      Yes │                                │              │
        │   │          ▼                                │              │
        │   │  ┌───────────────┐                       │              │
        │   │  │    착륙       │                       │              │
        │   │  └───────────────┘                       │              │
        │   └─────────────────────────────────────────┘              │
        └─────────────────────────────────────────────────────────────────┘
        """
        
        # ----------------------------------------------------------------------
        # 1단계: FCU 연결 대기
        # ----------------------------------------------------------------------
        while not rospy.is_shutdown() and not self.state.connected:
            rospy.loginfo("Waiting for FCU connection...")
            self.rate.sleep()
        """
        FCU (Flight Controller Unit) 연결 확인
        - Pixhawk 등의 비행 컨트롤러와 mavros 연결 대기
        - state.connected가 True가 될 때까지 루프
        """
        
        # ----------------------------------------------------------------------
        # 2단계: 초기 Setpoint 메시지 생성
        # ----------------------------------------------------------------------
        vel_cmd = TwistStamped()
        vel_cmd.twist.linear.x = 0
        vel_cmd.twist.linear.y = 0
        vel_cmd.twist.linear.z = 0
        """
        초기 속도 명령: 정지 (모든 축 0)
        
        [TwistStamped 구조]
        ┌──────────────────────────────────────┐
        │ TwistStamped                         │
        ├──────────────────────────────────────┤
        │ header:                              │
        │   stamp: 시간                        │
        │   frame_id: 좌표 프레임              │
        │ twist:                               │
        │   linear:  x, y, z (선형 속도)       │
        │   angular: x, y, z (각속도)          │
        └──────────────────────────────────────┘
        """

        # ----------------------------------------------------------------------
        # 3단계: 초기 Setpoint 100회 발행 (5초)
        # ----------------------------------------------------------------------
        for _ in range(100):
            vel_cmd.header.stamp = rospy.Time.now()
            self.cmd_pub.publish(vel_cmd)
            self.rate.sleep()
        """
        OFFBOARD 모드 전환 전 필수 조건
        
        [PX4 OFFBOARD 안전 메커니즘]
        1. 모드 전환 전: setpoint 스트림이 이미 존재해야 함
        2. 모드 전환 후: 2초 이상 setpoint 미수신 시 failsafe
        
        100회 × 50ms = 5초 동안 setpoint 발행
        → 안정적인 OFFBOARD 전환 보장
        
        [왜 100회인가?]
        - PX4는 최소 몇 개의 setpoint가 있어야 OFFBOARD 허용
        - 여유롭게 100회 설정 (실제로는 ~10회면 충분)
        - 통신 지연 등을 고려한 안전 마진
        """

        # ----------------------------------------------------------------------
        # 4단계: 드론 Arming
        # ----------------------------------------------------------------------
        self.arm()

        # ----------------------------------------------------------------------
        # 5단계: 목표 고도 설정
        # ----------------------------------------------------------------------
        target_alt = 1.0  # 목표 고도 [m]
        """
        착륙을 위한 목표 고도
        - 마커 위 1m 지점까지 하강
        - 정렬 완료 후 AUTO.LAND로 최종 착륙
        """

        # ----------------------------------------------------------------------
        # 6단계: 메인 제어 루프
        # ----------------------------------------------------------------------
        while not rospy.is_shutdown():
            vel_cmd = TwistStamped()
            vel_cmd.header.stamp = rospy.Time.now()

            if self.marker_detected:
                # ──────────────────────────────────────────────────────────────
                # 마커 검출 시: 카운터 증가 및 제어 실행
                # ──────────────────────────────────────────────────────────────
                self.marker_detected_count += 1

                if self.marker_detected_count >= self.required_detection_count:
                    # 50회 연속 검출 → OFFBOARD 전환 가능
                    
                    if self.state.mode != "OFFBOARD":
                        self.offboard_mode()

                    # ──────────────────────────────────────────────────────────
                    # PID 제어로 속도 계산
                    # ──────────────────────────────────────────────────────────
                    
                    # X축 속도 (전후): Y 오차로 제어 (카메라 Y = 드론 X)
                    vel_cmd.twist.linear.x = self.pid_y.compute(self.norm_y_error)
                    """
                    좌표계 매핑:
                    - 카메라 Y축 (상하) → 드론 X축 (전후)
                    - norm_y_error > 0: 마커가 이미지 아래 → 드론 전진 필요
                    """
                    
                    # Y축 속도 (좌우): X 오차로 제어 (카메라 X = 드론 Y)
                    vel_cmd.twist.linear.y = self.pid_x.compute(self.norm_x_error)
                    """
                    좌표계 매핑:
                    - 카메라 X축 (좌우) → 드론 Y축 (좌우)
                    - 동일 방향 (우측 = 우측)
                    """

                    # Z축 속도 (고도): 현재 고도와 목표 고도 차이
                    vel_cmd.twist.linear.z = self.pid_z.compute(self.my_alt)
                    """
                    고도 제어:
                    - pid_z.setpoint = 0 (기본값)
                    - error = 0 - my_alt = -my_alt
                    
                    예: 현재 고도 5m
                    → error = -5
                    → output = Kp × (-5) = 0.3 × (-5) = -1.5
                    → 출력 제한: -0.3 m/s (하강)
                    
                    참고: 목표 고도 1m로 설정하려면
                    pid_z.setpoint = target_alt 로 변경 필요
                    (현재 코드는 지면으로 하강하도록 설정됨)
                    """

                    # 디버깅 로그 출력
                    rospy.loginfo(f"Normalized Errors - X: {self.norm_x_error:.3f}, "
                                 f"Y: {self.norm_y_error:.3f}")
                    rospy.loginfo(f"Current Alt: {self.my_alt:.3f}m, "
                                 f"Target Alt: {target_alt:.3f}m")
                    rospy.loginfo(f"Velocities - X: {vel_cmd.twist.linear.x:.3f}, "
                                 f"Y: {vel_cmd.twist.linear.y:.3f}, "
                                 f"Z: {vel_cmd.twist.linear.z:.3f}")

                    # ──────────────────────────────────────────────────────────
                    # 착륙 조건 확인
                    # ──────────────────────────────────────────────────────────
                    alt_error = target_alt - self.my_alt
                    """
                    착륙 조건:
                    1. X 오차 < 5% (마커가 화면 중앙 ±5% 이내)
                    2. Y 오차 < 5%
                    3. 고도 오차 < 0.1m (목표 고도 ±10cm 이내)
                    
                    모든 조건 충족 시 AUTO.LAND 모드로 전환
                    """
                    
                    if (abs(self.norm_x_error) < 0.05 and 
                        abs(self.norm_y_error) < 0.05 and 
                        abs(alt_error) < 0.1):
                        rospy.loginfo("Position aligned, initiating landing...")
                        self.land()
                        break  # 루프 종료

            else:
                # ──────────────────────────────────────────────────────────────
                # 마커 미검출 시: 카운터 리셋 및 호버링
                # ──────────────────────────────────────────────────────────────
                self.marker_detected_count = 0
                
                # 수평 속도 0 (호버링)
                vel_cmd.twist.linear.x = 0.0
                vel_cmd.twist.linear.y = 0.0

                # 고도 제어는 계속 수행 (급격한 하강 방지)
                vel_cmd.twist.linear.z = self.pid_z.compute(self.my_alt)

            # 속도 명령 발행
            self.cmd_pub.publish(vel_cmd)
            
            # 주기 유지 (20Hz)
            self.rate.sleep()

    def __del__(self):
        """
        소멸자: OpenCV 윈도우 정리
        
        객체 소멸 시 호출되어 리소스 정리
        """
        cv2.destroyAllWindows()


# ==============================================================================
# 메인 실행부
# ==============================================================================

if __name__ == '__main__':
    """
    Python 스크립트 진입점
    
    __name__ == '__main__':
    - 이 파일이 직접 실행될 때만 True
    - import 될 때는 False → 아래 코드 실행 안 됨
    """
    try:
        # DroneController 인스턴스 생성
        controller = DroneController()
        
        # 초기 모드 설정 (선택적)
        # controller.set_mav_frame(1)  # MAV_FRAME_LOCAL_NED
        controller.set_mode("OFFBOARD")  # OFFBOARD 모드 사전 요청
        
        # 메인 제어 루프 시작
        controller.control_drone()
        
    except rospy.ROSInterruptException:
        """
        ROS 종료 시 발생하는 예외
        - Ctrl+C 입력
        - rosnode kill 명령
        - roslaunch 종료
        """
        pass