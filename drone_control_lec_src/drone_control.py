#!/usr/bin/env python3
import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool
from std_msgs.msg import Float64
from geometry_msgs.msg import TwistStamped, PoseStamped
from sensor_msgs.msg import CompressedImage, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf.transformations as transformations
import time

class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint=0.0, output_limits=(None, None)):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint

        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = None

        self.output_limits = output_limits

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = None

    def compute(self, measurement):
        current_time = time.time()
        error = self.setpoint - measurement

        delta_time = current_time - self._prev_time if self._prev_time else 0.0
        delta_error = error - self._prev_error if self._prev_time else 0.0

        if delta_time > 0.0:
            self._integral += error * delta_time
            derivative = delta_error / delta_time
        else:
            derivative = 0.0

        output = self.Kp * error + self.Ki * self._integral + self.Kd * derivative

        # Apply output limits
        lower, upper = self.output_limits
        if lower is not None:
            output = max(lower, output)
        if upper is not None:
            output = min(upper, output)

        self._prev_error = error
        self._prev_time = current_time

        return output

class DroneController: 
    def __init__(self):
        rospy.init_node("drone_controller")
        
        # 초기 변수 설정
        self.norm_x_error = 0.0  # 화면 중앙 기준 좌우 오차 (정규화: -1 ~ 1)
        self.norm_y_error = 0.0  # 화면 중앙 기준 상하 오차 (정규화: -1 ~ 1)
        self.z_error = 0.0       # ArUco 마커까지의 거리
        self.my_alt = 0.0  
        self.bridge = CvBridge()
        self.current_image = None
        self.marker_corners = None
        self.marker_ids = None
        self.camera_matrix = None
        self.dist_coeffs = None
        self.marker_size = 0.8  # 마커 실제 크기 (미터)
        self.distance = 0.0
        self.marker_detected = False

        # 현재 속도 변수
        self.current_vel_x = 0.0
        self.current_vel_y = 0.0
        self.current_vel_z = 0.0

        # 드론의 자세 변수
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        # ArUco 딕셔너리 및 파라미터 설정
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # PID 컨트롤러 설정
        self.pid_x = PIDController(Kp=0.5, Ki=0.0, Kd=0.1, output_limits=(-0.5, 0.5))
        self.pid_y = PIDController(Kp=0.5, Ki=0.0, Kd=0.1, output_limits=(-0.5, 0.5))
        self.pid_z = PIDController(Kp=0.3, Ki=0.0, Kd=0.05, output_limits=(-0.3, 0.3))

        # 서브스크라이버 설정
        self.camera_info_sub = rospy.Subscriber('/iris/usb_cam/camera_info', CameraInfo, self.camera_info_callback)
        self.altitude_sub = rospy.Subscriber("/mavros/global_position/rel_alt", Float64, self.altitude_callback)
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_callback)
        self.image_sub = rospy.Subscriber('/iris/usb_cam/image_raw/compressed', CompressedImage, self.image_callback)
        self.velocity_sub = rospy.Subscriber('/mavros/local_position/velocity_body', TwistStamped, self.velocity_callback)
        self.attitude_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.attitude_callback)
        self.state = State()

        # 퍼블리셔 설정
        self.cmd_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        self.rate = rospy.Rate(20)  # 20Hz

        # 제어 관련 카운터 및 플래그
        self.marker_detected_count = 0  
        self.required_detection_count = 50  # OFFBOARD 모드 전환을 위한 필요한 연속 마커 인식 횟수

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.K).reshape(3, 3)
        self.dist_coeffs = np.array(msg.D)

    def velocity_callback(self, msg):
        self.current_vel_x = msg.twist.linear.x
        self.current_vel_y = msg.twist.linear.y
        self.current_vel_z = msg.twist.linear.z

    def attitude_callback(self, msg):
        orientation_q = msg.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = transformations.euler_from_quaternion(orientation_list)

    def image_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.current_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if self.camera_matrix is not None:
                # 이미지 중심점 계산
                h, w = self.current_image.shape[:2]
                center_x, center_y = w//2, h//2

                corners, ids, rejected = cv2.aruco.detectMarkers(
                    self.current_image, 
                    self.aruco_dict, 
                    parameters=self.aruco_params
                )
                
                if ids is not None and len(ids) > 0:
                    # 가장 첫 번째 마커 사용
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                        corners, self.marker_size, 
                        self.camera_matrix, self.dist_coeffs
                    )
                    
                    # 마커의 화면상 중심점 계산
                    marker_center = np.mean(corners[0][0], axis=0)
                    
                    # 화면 중심으로부터의 픽셀 오차 계산
                    pixel_error_x = marker_center[0] - center_x  # 양수: 마커가 오른쪽에 있음
                    pixel_error_y = marker_center[1] - center_y  # 양수: 마커가 아래에 있음
                    
                    # 픽셀 오차를 정규화 (-1 ~ 1 범위로)
                    self.norm_x_error = pixel_error_x / (w/2)  # 우측이 양수
                    self.norm_y_error = pixel_error_y / (h/2)  # 아래가 양수
                    
                    # Z축 거리는 ArUco에서 직접 얻음 (미터 단위)
                    self.z_error = tvecs[0][0][2]
                    
                    self.marker_detected = True
                    self.distance = np.linalg.norm(tvecs[0][0])

                    # 시각화를 위한 마커 그리기
                    cv2.aruco.drawDetectedMarkers(self.current_image, corners, ids)
                    cv2.aruco.drawAxis(self.current_image, self.camera_matrix, self.dist_coeffs, 
                                     rvecs[0], tvecs[0], 0.1)
                    
                else:
                    self.norm_x_error = 0.0
                    self.norm_y_error = 0.0
                    self.z_error = 0.0
                    self.marker_detected = False

                self.marker_corners = corners
                self.marker_ids = ids
            
            self.display_image()
        except Exception as e:
            rospy.logerr(f"Error in image processing: {e}")

    def display_image(self):
        if self.current_image is not None:
            display_img = self.current_image.copy()
            h, w = display_img.shape[:2]
            center_x, center_y = w//2, h//2
            
            if self.marker_corners is not None and len(self.marker_corners) > 0:
                cv2.aruco.drawDetectedMarkers(display_img, self.marker_corners, self.marker_ids)
                
                for corner in self.marker_corners:
                    marker_center = np.mean(corner[0], axis=0).astype(int)
                    cv2.circle(display_img, tuple(marker_center), 5, (0,0,255), -1)
                    cv2.line(display_img, (center_x, center_y), tuple(marker_center), (255,0,0), 2)

            # 중심점 십자선 표시
            cv2.line(display_img, (center_x-20, center_y), (center_x+20, center_y), (0,255,0), 2)
            cv2.line(display_img, (center_x, center_y-20), (center_x, center_y+20), (0,255,0), 2)
            
            # 정보 표시
            info_text = [
                #f"Norm X Err: {self.norm_x_error:.3f}",
                #f"Norm Y Err: {self.norm_y_error:.3f}",
                #f"Z Distance: {self.z_error:.3f}m",
                f"Altitude: {self.my_alt:.2f}m",
                f"Roll: {np.degrees(self.roll):.2f}deg",
                f"Pitch: {np.degrees(self.pitch):.2f}deg",
                f"Yaw: {np.degrees(self.yaw):.2f}deg"
            ]

            for i, text in enumerate(info_text):
                cv2.putText(display_img, text, (10, 20 + i*20), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
            
            if self.marker_ids is not None and len(self.marker_ids) > 0:
                cv2.putText(display_img, f"Marker ID: {self.marker_ids[0]}", 
                           (10, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
            
            cv2.imshow("Drone View", display_img)
            cv2.waitKey(1)

    def altitude_callback(self, msg):
        self.my_alt = msg.data  

    def state_callback(self, msg):
        self.state = msg

    def arm(self):
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            response = arm_service(True)
            if response.success:
                rospy.loginfo("Vehicle armed")
            else:
                rospy.logerr("Failed to arm vehicle")
        except rospy.ServiceException as e:
            rospy.logerr("Arming failed: %s", str(e))

    def set_mav_frame(self, frame_id):
        rospy.wait_for_service('/mavros/setpoint_velocity/mav_frame')
        try:
            mav_frame_service = rospy.ServiceProxy('/mavros/setpoint_velocity/mav_frame', SetMode)
            # Note: SetMavFrame might not be a standard service; ensure it's correctly defined
            # This part may need adjustment based on actual service definitions
            # Here, assuming it's similar to SetMode
            response = mav_frame_service(frame_id)
            # Adjust based on actual response
            # Placeholder response handling
            rospy.loginfo("MAV frame set successfully to %d", frame_id)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

    def set_mode(self, mode):
        rospy.wait_for_service('/mavros/set_mode')
        try:
            set_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            response = set_mode_service(0, mode)
            if response.mode_sent:
                rospy.loginfo(f"Set mode to {mode}")
            else:
                rospy.logerr(f"Failed to set mode to {mode}")
        except rospy.ServiceException as e:
            rospy.logerr("Set mode failed: %s", str(e))

    def offboard_mode(self):
        if self.state.mode != "OFFBOARD":
            self.set_mode("OFFBOARD")
            rospy.loginfo("Offboard mode enabled.")

    def land(self):
        rospy.loginfo("Landing...")
        self.set_mode("AUTO.LAND")
        rospy.sleep(5)

    def control_drone(self):
        # FCU 연결 대기
        while not rospy.is_shutdown() and not self.state.connected:
            rospy.loginfo("Waiting for FCU connection...")
            self.rate.sleep()
        
        # 초기 setpoint 메시지 송신
        vel_cmd = TwistStamped()
        vel_cmd.twist.linear.x = 0
        vel_cmd.twist.linear.y = 0
        vel_cmd.twist.linear.z = 0

        # 초기 setpoint 메시지 100번 송신 (5초)
        for _ in range(100):
            vel_cmd.header.stamp = rospy.Time.now()
            self.cmd_pub.publish(vel_cmd)
            self.rate.sleep()

        # 드론 암링
        self.arm()

        # 목표 고도 설정 (고도 하강)
        target_alt = 1.0    # 목표 고도 (meters)

        while not rospy.is_shutdown():
            vel_cmd = TwistStamped()
            vel_cmd.header.stamp = rospy.Time.now()

            if self.marker_detected:
                self.marker_detected_count += 1

                if self.marker_detected_count >= self.required_detection_count:
                    if self.state.mode != "OFFBOARD":
                        self.offboard_mode()

                    # PID 컨트롤러를 사용하여 속도 계산
                    vel_cmd.twist.linear.x = self.pid_y.compute(self.norm_y_error)
                    vel_cmd.twist.linear.y = self.pid_x.compute(self.norm_x_error)

                    # 고도 제어
                    vel_cmd.twist.linear.z = self.pid_z.compute(self.my_alt)

                    # 자세 안정화 (롤 및 피치를 최소화)
                    # 여기서는 Roll과 Pitch를 직접 조정하지 않고, 속도 명령을 통해 안정화
                    # 추가적으로 자세 제어가 필요하다면 추가 구현 가능

                    rospy.loginfo(f"Normalized Errors - X: {self.norm_x_error:.3f}, Y: {self.norm_y_error:.3f}")
                    rospy.loginfo(f"Current Alt: {self.my_alt:.3f}m, Target Alt: {target_alt:.3f}m")
                    rospy.loginfo(f"Velocities - X: {vel_cmd.twist.linear.x:.3f}, " +
                                  f"Y: {vel_cmd.twist.linear.y:.3f}, Z: {vel_cmd.twist.linear.z:.3f}")

                    # 착륙 조건
                    alt_error = target_alt - self.my_alt
                    if (abs(self.norm_x_error) < 0.05 and 
                        abs(self.norm_y_error) < 0.05 and 
                        abs(alt_error) < 0.1):
                        rospy.loginfo("Position aligned, initiating landing...")
                        self.land()
                        break

            else:
                self.marker_detected_count = 0
                # 마커 미감지 시 호버링 및 고도 제어
                vel_cmd.twist.linear.x = 0.0
                vel_cmd.twist.linear.y = 0.0

                # 고도 제어
                vel_cmd.twist.linear.z = self.pid_z.compute(self.my_alt)

            self.cmd_pub.publish(vel_cmd)
            self.rate.sleep()

    def __del__(self):
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        controller = DroneController()
        # controller.set_mav_frame(1)  # MAV_FRAME_LOCAL_NED = 1 (세계 프레임 기준)
        controller.set_mode("OFFBOARD")  # Ensure the drone is in OFFBOARD mode
        controller.control_drone()
    except rospy.ROSInterruptException:
        pass
