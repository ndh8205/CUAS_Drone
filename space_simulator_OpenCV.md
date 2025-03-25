아래는 시스템 전역 설치 방식(apt를 이용하여 python3-opencv 설치)을 반영한 업데이트된 **ArUco 마커 인식 노드 구축 가이드**입니다.

---

# 🚀 WSL2 Ubuntu 24.04에서 ArUco 마커 인식 노드 구축 가이드  
**환경:** Windows 10/11 (WSL2), Ubuntu 24.04, ROS 2 Jazzy  
**목표:** 인공위성 시뮬레이션의 카메라 센서(`/nasa_satellite/camera`)를 구독하여, OpenCV의 ArUco 모듈을 통해 실시간으로 마커 인식을 수행하는 ROS 2 노드를 구축  
**비고:** 본 가이드는 ROS 2 Python 노드, OpenCV, cv_bridge를 활용하여 ArUco 마커 검출 기능을 추가하는 데 중점을 둡니다.

---

## 📌 목차
1. [필수 패키지 및 추가 의존성 설치](#1-필수-패키지-및-추가-의존성-설치)
2. [ROS 2 패키지 생성](#2-ros-2-패키지-생성)
3. [ArUco 마커 인식 노드 코드 작성](#3-aruco-마커-인식-노드-코드-작성)
4. [빌드 및 실행](#4-빌드-및-실행)
5. [추가 참고 자료](#5-추가-참고-자료)

---

## 1. 필수 패키지 및 추가 의존성 설치

### 1.1 OpenCV 및 cv_bridge 설치
WSL2 환경에서 ROS 2 Jazzy와 연동하여 카메라 이미지를 처리하기 위해, 시스템 전역에 apt를 사용하여 아래 패키지를 설치합니다.  
터미널에서 다음 명령어를 실행하세요:
```bash
sudo apt update
sudo apt install -y ros-jazzy-cv-bridge python3-opencv
```
*참고:* pip로 설치 시 PEP 668에 따라 "externally managed environment" 오류가 발생할 수 있으므로, 시스템 전역 설치를 권장합니다.

---

## 2. ROS 2 패키지 생성

### 2.1 패키지 생성
ROS 2 Python 패키지를 생성하여 ArUco 마커 인식 노드를 작성합니다.  
터미널에서 아래 명령어를 실행합니다:
```bash
ros2 pkg create --build-type ament_python aruco_detection
```

### 2.2 파일 구조
생성된 패키지의 기본 파일 구조는 다음과 같습니다:
```
aruco_detection/
├── package.xml
├── setup.py
└── aruco_detection
    └── __init__.py
```

### 2.3 `setup.py` 및 `package.xml` 수정
패키지 의존성을 추가하기 위해 아래와 같이 파일을 수정합니다.

#### setup.py 예시:
```python
from setuptools import setup

package_name = 'aruco_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='작성자 이름',
    author_email='your.email@example.com',
    maintainer='작성자 이름',
    maintainer_email='your.email@example.com',
    description='ROS2 패키지를 활용한 ArUco 마커 인식 노드',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_detector = aruco_detection.aruco_detector:main'
        ],
    },
)
```

#### package.xml 예시:
```xml
<package format="3">
  <name>aruco_detection</name>
  <version>0.0.0</version>
  <description>ROS2 패키지를 활용한 ArUco 마커 인식 노드</description>
  <maintainer email="your.email@example.com">작성자 이름</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>ament_python</buildtool_depend>

  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>
  <depend>cv_bridge</depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

---

## 3. ArUco 마커 인식 노드 코드 작성

패키지 내 `aruco_detection/aruco_detector.py` 파일을 생성하고 아래 코드를 작성합니다.

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import cv2.aruco as aruco

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.subscription = self.create_subscription(
            Image,
            'nasa_satellite/camera',  # Gazebo 시뮬레이션에서 발행하는 카메라 센서 토픽
            self.image_callback,
            10)
        self.bridge = CvBridge()
        # 사용할 ArUco 사전 선택 (예: DICT_6X6_250)
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters_create()

    def image_callback(self, msg):
        try:
            # ROS 이미지 메시지를 OpenCV 이미지(BGR8)로 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge error: {e}")
            return

        # ArUco 마커 검출
        corners, ids, _ = aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.parameters)
        if ids is not None:
            # 검출된 마커에 사각형 그리기
            aruco.drawDetectedMarkers(cv_image, corners, ids)
            self.get_logger().info(f"Detected markers: {ids.flatten().tolist()}")
        else:
            self.get_logger().info("No markers detected")
        
        # 결과 이미지 표시 (X서버 설정 필요)
        cv2.imshow("Aruco Detection", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    detector = ArucoDetector()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

*설명:*  
- **이미지 구독:** `/nasa_satellite/camera` 토픽에서 이미지를 받아 cv_bridge를 통해 OpenCV 이미지로 변환합니다.  
- **ArUco 검출:** OpenCV의 `cv2.aruco.detectMarkers` 함수를 이용해 이미지 내 마커를 검출하고, 검출된 마커에 사각형을 그려 결과를 표시합니다.  
- **결과 활용:** 검출된 마커 ID를 로그로 출력하며, 결과 이미지는 `cv2.imshow`를 통해 확인할 수 있습니다.

---

## 4. 빌드 및 실행

### 4.1 빌드
작업공간의 `src` 디렉토리에 생성한 `aruco_detection` 패키지를 포함하여 전체 빌드를 진행합니다.
```bash
colcon build --symlink-install
```

### 4.2 환경 설정
빌드가 완료되면 아래 명령어로 환경 변수를 설정합니다.
```bash
source install/setup.bash
```

### 4.3 실행
ROS 2 노드를 실행하여 ArUco 마커 인식 기능을 확인합니다.
```bash
ros2 run aruco_detection aruco_detector
```

*참고:*  
- Gazebo Harmonic에서 카메라 센서가 정상적으로 `/nasa_satellite/camera` 토픽으로 이미지를 발행 중인지 확인하세요.  
- X서버(VcXsrv 등)를 통해 GUI 창(`cv2.imshow`)이 제대로 표시되는지 점검합니다.

---

## 5. 추가 참고 자료

- [ROS 2 공식 문서](https://docs.ros.org/)
- [OpenCV ArUco 튜토리얼](https://docs.opencv.org/master/d5/dae/tutorial_aruco_detection.html)
- [cv_bridge 사용법](https://wiki.ros.org/cv_bridge)
- [Gazebo Harmonic 공식 문서](https://gazebosim.org/docs/harmonic)
- [WSL2에서 GUI 설정 (VcXsrv)](https://sourceforge.net/projects/vcxsrv/)

---

이 가이드를 통해, ROS 2 Jazzy 환경에서 인공위성 시뮬레이션의 카메라 센서를 활용해 ArUco 마커를 실시간 인식하는 노드를 구축할 수 있습니다. 추가 질문이나 수정 사항이 있으면 언제든지 문의해 주세요!
