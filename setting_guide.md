# Jetson Orin Nano 시스템 확인 가이드

호스트 PC(Jetson이 아닌 Ubuntu 20.04가 설치되어있는 컴퓨터)에서 SDK Manager를 통한 설치

1. 호스트 PC 요구사항
```
OS: Ubuntu 20.04
메모리: 최소 8GB RAM (16GB 권장)
저장공간: 최소 40GB 여유 공간
인터넷: 안정적인 인터넷 연결
```
2. SDK Manager 설치
```
https://developer.nvidia.com 에서 계정 생성
```
### 2. SDK Manager 다운로드
```
 https://developer.nvidia.com/nvidia-sdk-manager 에서 
 sdkmanager_[version]_amd64.deb 파일 다운로드
```
### 3. SDK Manager 설치
```
sudo apt update
sudo apt install ./sdkmanager_*_amd64.deb
```
### 4. 의존성 패키지 설치
```
sudo apt install -y libgconf-2-4 libcanberra-gtk-module
```

### 5. Jetson orin nano 연결

### 6. Jetson SDK에서 jetpack 설치 진행



## 시스템 업데이트 및 정보 확인 - Jetson Orin Nano

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
### (3) 카메라 연결 및 작동 확인

장치 파일이 있는지 확인합니다.
```bash
ls /dev/video*
```
카메라 테스트를 위해 `cheese`프로그램을 사용.
```bash
sudo apt-get install cheese
cheese
```

### (3) OpenCV 및 ArUco 모듈 확인

#### OpenCV 버전 확인
터미널에서 다음 명령어를 실행하여 OpenCV 버전을 확인합니다 (OpenCV 4.x 기준).
```bash
pkg-config --modversion opencv4
```
만약 위 명령어로 버전이 나오지 않거나 OpenCV 3.x 버전이 설치되어 있다면, `opencv`로 시도해볼 수 있습니다.
```bash
pkg-config --modversion opencv
```

#### ArUco 모듈 포함 여부 확인
ArUco 모듈 (`aruco.hpp`)이 현재 설치된 OpenCV에 포함되어 있는지 확인합니다. 다음 테스트 코드를 작성하여 컴파일해봅니다.

`test_aruco.cpp` 파일을 생성하고 아래 내용을 입력합니다:
```cpp
#include <opencv2/aruco.hpp>
#include <iostream>

int main() {
    std::cout << "OpenCV ArUco module is available!" << std::endl;
    return 0;
}
```
컴파일 및 실행:
```bash
g++ -std=c++11 -o test_aruco test_aruco.cpp $(pkg-config --cflags --libs opencv4)
./test_aruco
```
"OpenCV ArUco module is available!" 메시지가 출력되면 ArUco 모듈이 정상적으로 포함된 것입니다. 그렇지 않다면 아래 "4. OpenCV (opencv_contrib) 설치 방법 (필요시)" 섹션을 참고하여 OpenCV를 재설치해야 합니다.

### (4) C++ 컴파일러 (g++) 확인

터미널에서 g++ 버전을 확인하여 C++17 이상을 지원하는지 확인합니다.
```bash
g++ --version
```

### (5) 필수 종속 패키지 설치 확인
OpenCV 및 GUI 표시 등에 필요한 라이브러리들을 설치합니다.
```bash
sudo apt-get update
sudo apt-get install build-essential cmake git libgtk-3-dev libcanberra-gtk3-module
sudo apt-get install libjpeg-dev libpng-dev libtiff-dev
sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get install libv4l-dev
```
이 명령어들을 실행하면 누락된 필수 라이브러리들이 설치됩니다.

---

## 4. OpenCV (opencv_contrib) 설치 방법 (필요시) 🛠️

만약 Jetson에 설치된 OpenCV에 ArUco 모듈이 없거나 특정 버전의 OpenCV를 사용해야 하는 경우, 소스에서 `opencv_contrib`와 함께 빌드해야 합니다.

1.  **필요한 패키지 설치**:
    ```bash
    sudo apt-get update
    sudo apt-get install build-essential cmake git libgtk-3-dev libcanberra-gtk3-module \
                         libjpeg-dev libpng-dev libtiff-dev \
                         libavcodec-dev libavformat-dev libswscale-dev \
                         libv4l-dev
    # CUDA 관련 옵션을 사용하려면 libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev 등도 필요할 수 있습니다.
    ```

2.  **OpenCV 및 opencv_contrib 소스 클론**:
    원하는 버전으로 태그를 체크아웃할 수 있습니다. (예: `git checkout 4.2.0`)
    ```bash
    cd ~
    git clone [https://github.com/opencv/opencv.git](https://github.com/opencv/opencv.git)
    git clone [https://github.com/opencv/opencv_contrib.git](https://github.com/opencv/opencv_contrib.git)
    # 버전 일치를 위해 opencv와 opencv_contrib 모두 동일한 버전 태그로 체크아웃하는 것이 좋습니다.
    # cd opencv && git checkout <원하는 버전> && cd ..
    # cd opencv_contrib && git checkout <원하는 버전> && cd ..
    ```

3.  **빌드 디렉토리 생성 및 CMake 설정**:
    ```bash
    cd ~/opencv
    mkdir build && cd build

    cmake -D CMAKE_BUILD_TYPE=Release \
          -D CMAKE_INSTALL_PREFIX=/usr/local \
          -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules \
          -D WITH_CUDA=ON \
          -D CUDA_ARCH_BIN="<Jetson_Compute_Capability>" \ # 예: "5.3,6.2,7.2,8.7" (Jetson 모델에 맞게 설정)
          -D CUDA_ARCH_PTX="" \
          -D WITH_CUDNN=ON \
          -D OPENCV_DNN_CUDA=ON \
          -D ENABLE_FAST_MATH=1 \
          -D CUDA_FAST_MATH=1 \
          -D WITH_CUBLAS=1 \
          -D WITH_GTK=ON \
          -D WITH_OPENGL=ON \
          -D BUILD_EXAMPLES=OFF \
          -D BUILD_TESTS=OFF \
          -D BUILD_PERF_TESTS=OFF \
          ..
    ```
    * `CUDA_ARCH_BIN`: Jetson 모델의 Compute Capability를 확인하여 설정합니다. (예: Orin Nano는 8.7)
        * Jetson Nano: 5.3
        * Jetson TX2: 6.2
        * Jetson Xavier NX: 7.2
        * Jetson Orin Nano/NX/AGX: 8.7

4.  **빌드 및 설치**:
    Jetson 보드의 성능에 따라 시간이 매우 오래 걸릴 수 있습니다. (수 시간 소요 가능)
    ```bash
    make -j$(nproc)
    sudo make install
    ```

5.  **설치 확인**:
    ```bash
    pkg-config --modversion opencv4
    ```
    설치 후, 필요하다면 `.bashrc`에 라이브러리 경로를 추가하거나 `ldconfig`를 실행합니다.
    ```bash
    sudo ldconfig
    ```

---

## 5. C++ 코드 준비 📝

`apriltag_pose_estimation.cpp`와 같이 파일 이름으로 저장합니다.

```cpp
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <vector>
#include <stdexcept>
#include <cmath>

// 구조체: Pose (R, t, H)
struct Pose {
    cv::Mat R; // 3x3 회전 행렬 (CV_64F)
    cv::Mat t; // 3x1 병진 벡터 (CV_64F)
    cv::Mat H; // 3x3 호모그래피 행렬 (CV_64F)
};

// 구조체: PoseError (R, t, 재투영 오차)
struct PoseError {
    cv::Mat R;
    cv::Mat t;
    double error;
};

// 회전 행렬을 Euler 각도로 변환 (deg 단위)
cv::Vec3d rotationMatrixToEulerAngles(const cv::Mat &R) {
    double sy = std::sqrt(R.at<double>(0,0)*R.at<double>(0,0) + R.at<double>(1,0)*R.at<double>(1,0));
    bool singular = sy < 1e-6;
    double x, y, z;
    if (!singular) {
        x = std::atan2(R.at<double>(2,1), R.at<double>(2,2));
        y = std::atan2(-R.at<double>(2,0), sy);
        z = std::atan2(R.at<double>(1,0), R.at<double>(0,0));
    } else {
        x = std::atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = std::atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    return cv::Vec3d(x * 180.0 / CV_PI, y * 180.0 / CV_PI, z * 180.0 / CV_PI);
}

// 4개 점을 좌상단, 우상단, 우하단, 좌하단 순서로 정렬
std::vector<cv::Point2f> orderPoints(const std::vector<cv::Point2f>& pts) {
    if (pts.size() != 4) {
        throw std::runtime_error("포인트 수가 4개가 아닙니다.");
    }
    std::vector<cv::Point2f> rect(4);
    std::vector<float> sums, diffs;
    for (const auto &pt : pts) {
        sums.push_back(pt.x + pt.y);
        diffs.push_back(pt.x - pt.y);
    }
    auto minSumIter = std::min_element(sums.begin(), sums.end());
    auto maxSumIter = std::max_element(sums.begin(), sums.end());
    auto minDiffIter = std::min_element(diffs.begin(), diffs.end());
    auto maxDiffIter = std::max_element(diffs.begin(), diffs.end());

    rect[0] = pts[std::distance(sums.begin(), minSumIter)]; // 좌상단
    rect[1] = pts[std::distance(diffs.begin(), minDiffIter)]; // 우상단
    rect[2] = pts[std::distance(sums.begin(), maxSumIter)]; // 우하단
    rect[3] = pts[std::distance(diffs.begin(), maxDiffIter)]; // 좌하단
    return rect;
}

// 재투영 오차 계산 (평균 Euclidean distance)
double computeReprojectionError(const std::vector<cv::Point3f>& object_pts,
                                const std::vector<cv::Point2f>& image_pts,
                                const cv::Mat& rvec, const cv::Mat& tvec,
                                const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs) {
    if (object_pts.empty() || image_pts.empty()) return -1.0; // 유효성 검사
    std::vector<cv::Point2f> projected;
    cv::projectPoints(object_pts, rvec, tvec, camera_matrix, dist_coeffs, projected);
    double error = 0.0;
    if (projected.size() != image_pts.size()) return -1.0; // 유효성 검사
    for (size_t i = 0; i < image_pts.size(); i++) {
        error += cv::norm(projected[i] - image_pts[i]);
    }
    return error / image_pts.size();
}

// 호모그래피 분해를 통한 초기 해 산출
Pose robustHomographyDecomposition(const std::vector<cv::Point2f>& model_pts_2d,
                                   const std::vector<cv::Point2f>& ordered_corners,
                                   const cv::Mat& camera_matrix) {
    Pose pose;
    pose.H = cv::getPerspectiveTransform(model_pts_2d, ordered_corners);
    cv::Mat K_inv = camera_matrix.inv();
    cv::Mat h1 = pose.H.col(0);
    cv::Mat h2 = pose.H.col(1);
    cv::Mat h3 = pose.H.col(2);

    cv::Mat K_inv_h1 = K_inv * h1;
    double norm_h1 = cv::norm(K_inv_h1);
    if (norm_h1 < 1e-8) { // 안정성을 위해 1e-6에서 1e-8로 조정
        throw std::runtime_error("호모그래피 분해 불안정: K_inv * h1의 노름이 너무 작음");
    }
    double lambda_val = 1.0 / norm_h1;
    cv::Mat r1 = lambda_val * (K_inv * h1);
    cv::Mat r2 = lambda_val * (K_inv * h2);
    cv::Mat t_init = lambda_val * (K_inv * h3);

    cv::Vec3d r1_vec(r1.at<double>(0), r1.at<double>(1), r1.at<double>(2));
    cv::Vec3d r2_vec(r2.at<double>(0), r2.at<double>(1), r2.at<double>(2));
    cv::Vec3d r3_vec = r1_vec.cross(r2_vec);
    cv::Mat r3_mat = (cv::Mat_<double>(3, 1) << r3_vec[0], r3_vec[1], r3_vec[2]); // 변수명 변경

    cv::Mat R_init(3, 3, CV_64F);
    r1.copyTo(R_init.col(0));
    r2.copyTo(R_init.col(1));
    r3_mat.copyTo(R_init.col(2)); // 변경된 변수명 사용

    cv::Mat U, W, Vt;
    cv::SVD::compute(R_init, W, U, Vt);
    pose.R = U * Vt;
    pose.t = t_init.clone();

    return pose;
}

// Orthogonal Iteration을 통한 R, t 최적화
std::pair<cv::Mat, cv::Mat> orthogonalIteration(const cv::Mat& R_init, const cv::Mat& t_init,
                                                const std::vector<cv::Point3f>& object_pts,
                                                const std::vector<cv::Vec3d>& v_list,
                                                int n_iters = 20) {
    cv::Mat R_iter = R_init.clone();
    cv::Mat t_iter = t_init.clone();
    for (int iter = 0; iter < n_iters; iter++) {
        // (a) t 업데이트
        cv::Mat A = cv::Mat::zeros(3, 3, CV_64F);
        cv::Mat b = cv::Mat::zeros(3, 1, CV_64F);
        for (size_t i = 0; i < object_pts.size(); i++) {
            cv::Mat v_mat = (cv::Mat_<double>(3, 1) << v_list[i][0], v_list[i][1], v_list[i][2]);
            double norm_v2 = v_list[i].dot(v_list[i]);
            if (norm_v2 < 1e-8) norm_v2 = 1e-8; // 0으로 나누기 방지
            cv::Mat F_i = (v_mat * v_mat.t()) / norm_v2;
            cv::Mat I_minus_F = cv::Mat::eye(3, 3, CV_64F) - F_i;
            cv::Mat p_mat = (cv::Mat_<double>(3, 1) << object_pts[i].x, object_pts[i].y, object_pts[i].z);
            A += I_minus_F;
            cv::Mat Rp = R_iter * p_mat;
            b += -I_minus_F * Rp;
        }
        cv::solve(A, b, t_iter, cv::DECOMP_SVD);

        // (b) R 업데이트
        cv::Mat Q_pts(object_pts.size(), 3, CV_64F); // 변수명 변경 Q -> Q_pts
        cv::Mat P_pts(object_pts.size(), 3, CV_64F); // 변수명 변경 P -> P_pts
        for (size_t i = 0; i < object_pts.size(); i++) {
            cv::Mat v_mat = (cv::Mat_<double>(3, 1) << v_list[i][0], v_list[i][1], v_list[i][2]);
             double norm_v2 = v_list[i].dot(v_list[i]);
            if (norm_v2 < 1e-8) norm_v2 = 1e-8; // 0으로 나누기 방지
            cv::Mat F_i = (v_mat * v_mat.t()) / norm_v2;
            cv::Mat p_mat = (cv::Mat_<double>(3, 1) << object_pts[i].x, object_pts[i].y, object_pts[i].z);
            cv::Mat X_i = R_iter * p_mat + t_iter;
            cv::Mat q_i = F_i * X_i;
            for (int j = 0; j < 3; j++) {
                Q_pts.at<double>(i, j) = q_i.at<double>(j, 0);
                P_pts.at<double>(i, j) = p_mat.at<double>(j, 0);
            }
        }
        cv::Mat Q_mean, P_mean;
        cv::reduce(Q_pts, Q_mean, 0, cv::REDUCE_AVG);
        cv::reduce(P_pts, P_mean, 0, cv::REDUCE_AVG);
        cv::Mat Q_centered = Q_pts - cv::repeat(Q_mean, Q_pts.rows, 1);
        cv::Mat P_centered = P_pts - cv::repeat(P_mean, P_pts.rows, 1);
        cv::Mat H_cov = P_centered.t() * Q_centered;
        cv::Mat U, S, Vt;
        cv::SVD::compute(H_cov, S, U, Vt);
        cv::Mat R_new = Vt.t() * U.t();
        if (cv::determinant(R_new) < 0) {
            Vt.row(2) *= -1;
            R_new = Vt.t() * U.t();
        }
        R_iter = R_new.clone();
    }
    return { R_iter, t_iter };
}

// 모호성 해결: 태그 법선(세번째 열) 뒤집기
PoseError fixPoseAmbiguities(const cv::Mat& R, const cv::Mat& t,
                             const std::vector<cv::Point3f>& object_pts,
                             const std::vector<cv::Point2f>& ordered_corners,
                             const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs) {
    PoseError result;
    cv::Mat R_flipped = R.clone();
    if (R_flipped.cols >=3) { // 유효성 검사
      R_flipped.col(2) = -R_flipped.col(2); // Z축 반전
      // X축, Y축도 반전시켜서 오른손 좌표계 유지 시도
      R_flipped.col(0) = -R_flipped.col(0); 
      // R_flipped.col(1) = -R_flipped.col(1); // Y축은 보통 그대로 둠
    } else {
        result.R = R.clone();
        result.t = t.clone();
        result.error = 1e9; // 매우 큰 오류 값
        return result;
    }


    cv::Mat rvec_orig, rvec_flip;
    cv::Rodrigues(R, rvec_orig);
    cv::Rodrigues(R_flipped, rvec_flip);
    double error_orig = computeReprojectionError(object_pts, ordered_corners, rvec_orig, t, camera_matrix, dist_coeffs);
    double error_flip = computeReprojectionError(object_pts, ordered_corners, rvec_flip, t, camera_matrix, dist_coeffs);

    if (error_flip < error_orig && error_flip >= 0) { // error_flip이 유효할 때
        result.R = R_flipped.clone();
        result.t = t.clone();
        result.error = error_flip;
    } else {
        result.R = R.clone();
        result.t = t.clone();
        result.error = error_orig;
    }
    return result;
}


// AprilTag 포즈 추정 함수 (ArUco 코너 입력)
std::pair<cv::Mat, cv::Mat> apriltagPoseEstimation(const std::vector<cv::Point2f>& corners_in, // 변수명 변경
                                                   const cv::Mat& camera_matrix,
                                                   double marker_length,
                                                   const cv::Mat& dist_coeffs_in) { // 변수명 변경
    cv::Mat distCoeffs = dist_coeffs_in; // 내부에서 복사해서 사용
    if (distCoeffs.empty()) {
        distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
    }

    // 1. 코너 순서 정렬
    std::vector<cv::Point2f> ordered_corners = orderPoints(corners_in);

    // 2. 모델 좌표 (태그 중심 기준 정사각형)
    double s = marker_length / 2.0;
    std::vector<cv::Point2f> model_pts_2d = {
        cv::Point2f(-s, s),
        cv::Point2f( s, s),
        cv::Point2f( s, -s),
        cv::Point2f(-s, -s)
    };
    std::vector<cv::Point3f> object_pts = {
        cv::Point3f(-s,  s, 0), // top-left
        cv::Point3f( s,  s, 0), // top-right
        cv::Point3f( s, -s, 0), // bottom-right
        cv::Point3f(-s, -s, 0)  // bottom-left
    };

    // 3. 강건한 호모그래피 분해 (fallback: 예외 발생 시 solvePnP 사용)
    cv::Mat R_init, t_init;
    try {
        Pose initPose = robustHomographyDecomposition(model_pts_2d, ordered_corners, camera_matrix);
        R_init = initPose.R;
        t_init = initPose.t;
    }
    catch (const std::exception &e) {
        std::cerr << "강건한 호모그래피 분해 실패: " << e.what() << ". solvePnP 사용." << std::endl;
        cv::Mat rvec, tvec;
        bool ok = cv::solvePnP(object_pts, ordered_corners, camera_matrix, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_IPPE); // IPPE 시도
        if (!ok) {
             ok = cv::solvePnP(object_pts, ordered_corners, camera_matrix, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE); // ITERATIVE 시도
             if (!ok) throw std::runtime_error("solvePnP 실패");
        }
        cv::Rodrigues(rvec, R_init);
        t_init = tvec.clone();
    }

    // 4. 각 코너의 정규화된 시선 벡터 계산
    double fx = camera_matrix.at<double>(0, 0);
    double fy = camera_matrix.at<double>(1, 1);
    double cx = camera_matrix.at<double>(0, 2);
    double cy = camera_matrix.at<double>(1, 2);
    std::vector<cv::Vec3d> v_list;
    for (const auto &pt : ordered_corners) {
        double u = pt.x, v = pt.y;
        cv::Vec3d vec((u - cx) / fx, (v - cy) / fy, 1.0);
        cv::normalize(vec, vec); // 정규화 추가
        v_list.push_back(vec);
    }

    // 5. Orthogonal Iteration을 통한 최적화
    auto [R_opt, t_opt] = orthogonalIteration(R_init, t_init, object_pts, v_list, 20);

    // 후보 해 1의 재투영 오차 계산
    cv::Mat rvec_candidate1;
    cv::Rodrigues(R_opt, rvec_candidate1);
    double error_candidate1 = computeReprojectionError(object_pts, ordered_corners, rvec_candidate1, t_opt, camera_matrix, distCoeffs);

    // 6. 모호성 해결
    PoseError candidate2 = fixPoseAmbiguities(R_opt, t_opt, object_pts, ordered_corners, camera_matrix, distCoeffs);

    cv::Mat best_R, best_t;
    if (candidate2.error < error_candidate1 && candidate2.error >= 0) {
        best_R = candidate2.R.clone();
        best_t = candidate2.t.clone();
        // std::cout << "Ambiguity fixed. Error: " << candidate2.error << std::endl;
    } else {
        best_R = R_opt.clone();
        best_t = t_opt.clone();
        // std::cout << "Original pose chosen. Error: " << error_candidate1 << std::endl;
    }
    
    // 추가 검증: t_z가 음수이면 R_z, R_y 180도 회전 (카메라 뒤에 있는 경우)
    if (best_t.at<double>(2) < 0) {
        cv::Mat R_z_180 = (cv::Mat_<double>(3,3) << -1, 0, 0, 0, -1, 0, 0, 0, 1);
        best_R = best_R * R_z_180;
        best_t = -best_t; // t_z를 양수로
    }

    return { best_R, best_t };
}

int main() {
    // 카메라 캘리브레이션 파라미터 (실제 값으로 교체)
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 621.941698, 0, 32.103575,
                                                      0, 620.436755, 242.676063,
                                                      0, 0, 1);
    // 실제 왜곡 계수 (칼리브레이션 값을 이용하여 추출)
    cv::Mat dist_coeffs = (cv::Mat_<double>(5, 1) << 0.060787, -0.138339, -0.003005, -0.001077, 0.000000);
    // cv::Mat dist_coeffs = cv::Mat::zeros(5, 1, CV_64F); // 왜곡 계수를 사용하지 않으려면 0으로 설정

    double marker_length = 0.035;  // 마커 실제 길이 (미터 단위, 예: 3.5cm = 0.035m)

    // ArUco 사전 선택 및 DetectorParameters 객체 생성
    // cv::aruco::getPredefinedDictionary는 cv::aruco::Dictionary 객체를 반환 (OpenCV 4.x 기준)
    // 이를 cv::Ptr로 감싸줘야 detectMarkers 함수와 호환됨.
    cv::aruco::Dictionary dict_obj = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_100); // 예시로 DICT_6X6_100 사용
    cv::Ptr<cv::aruco::Dictionary> aruco_dict = cv::makePtr<cv::aruco::Dictionary>(dict_obj);
    
    // DetectorParameters 생성 (OpenCV 버전에 따라 create() 또는 makePtr 사용)
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::makePtr<cv::aruco::DetectorParameters>();
    // parameters->adaptiveThreshWinSizeMin = 3;
    // parameters->adaptiveThreshWinSizeMax = 23;
    // parameters->adaptiveThreshWinSizeStep = 10;
    // parameters->adaptiveThreshConstant = 7;
    // parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX; // 코너 개선 방법

    cv::VideoCapture cap(0); // 0번 카메라 사용. CSI 카메라는 다른 번호나 GStreamer 파이프라인 문자열 사용
    if (!cap.isOpened()) {
        std::cerr << "웹캠을 열 수 없습니다." << std::endl;
        return -1;
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);  // 해상도 설정
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480); // 해상도 설정
    // cap.set(cv::CAP_PROP_FPS, 30); // FPS 설정

    cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001);

    while (true) {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "프레임을 가져올 수 없습니다." << std::endl;
            break;
        }

        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // ArUco 마커 검출
        std::vector<std::vector<cv::Point2f>> corners;
        std::vector<int> ids;
        cv::aruco::detectMarkers(gray, aruco_dict, corners, ids, parameters);

        cv::Mat result_frame = frame.clone();

        if (!ids.empty()) {
             // 코너 서브픽셀 정밀도 향상 (선택 사항)
            for (size_t i = 0; i < corners.size(); i++) {
                cv::cornerSubPix(gray, corners[i], cv::Size(5, 5), cv::Size(-1,-1), criteria);
            }
            cv::aruco::drawDetectedMarkers(result_frame, corners, ids);

            // 각 마커에 대해 포즈 추정
            for (size_t i = 0; i < ids.size(); i++) {
                std::vector<cv::Point2f> marker_corners = corners[i];
                cv::Mat R, t;
                try {
                    std::tie(R, t) = apriltagPoseEstimation(marker_corners, camera_matrix, marker_length, dist_coeffs);
                    
                    // 포즈 그리기 (drawFrameAxes는 OpenCV 3.x의 aruco 모듈 함수, OpenCV 4.x에서는 cv::drawFrameAxes)
                    // cv::aruco::drawAxis(result_frame, camera_matrix, dist_coeffs, R, t, marker_length * 0.5f); // OpenCV 3.x
                    cv::drawFrameAxes(result_frame, camera_matrix, dist_coeffs, R, t, marker_length * 0.5f); // OpenCV 4.x
                    
                    cv::Vec3d euler_angles = rotationMatrixToEulerAngles(R);
                    double roll = euler_angles[0], pitch = euler_angles[1], yaw = euler_angles[2];
                    double t_x = t.at<double>(0), t_y = t.at<double>(1), t_z = t.at<double>(2);
                    
                    int text_x_offset = 10;
                    int text_y_offset = 30 + static_cast<int>(i * 100); // 마커별로 텍스트 위치 조정

                    cv::putText(result_frame, cv::format("ID: %d", ids[i]), cv::Point(text_x_offset, text_y_offset),
                                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
                    cv::putText(result_frame, cv::format("Roll: %.1f", roll), cv::Point(text_x_offset, text_y_offset + 20),
                                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);
                    cv::putText(result_frame, cv::format("Pitch: %.1f", pitch), cv::Point(text_x_offset, text_y_offset + 40),
                                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
                    cv::putText(result_frame, cv::format("Yaw: %.1f", yaw), cv::Point(text_x_offset, text_y_offset + 60),
                                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);
                    cv::putText(result_frame, cv::format("t: [%.2f, %.2f, %.2f]m", t_x, t_y, t_z),
                                cv::Point(text_x_offset, text_y_offset + 80), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 2);

                } catch (const std::exception &e) {
                    std::cerr << "ID " << ids[i] << " 포즈 추정 실패: " << e.what() << std::endl;
                    continue;
                }
            }
        } else {
            cv::putText(result_frame, "No marker detected", cv::Point(10, 30),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
        }

        cv::imshow("ArUco Pose Estimation", result_frame);
        if (cv::waitKey(1) == 'q')
            break;
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
```
**주요 수정 및 확인 사항:**
* `main()` 함수 내의 카메라 매개변수 (`camera_matrix`, `dist_coeffs`)와 마커 길이 (`marker_length`)는 사용하는 카메라와 마커에 맞게 **반드시 수정**해야 합니다.
* ArUco 사전 (`aruco_dict`)은 `cv::aruco::getPredefinedDictionary(...)`로 얻은 객체를 `cv::makePtr<cv::aruco::Dictionary>(dict_obj)`를 사용하여 `cv::Ptr`로 감싸줍니다. (OpenCV 버전에 따라 `new cv::aruco::Dictionary(temp_dict)` 방식이 필요할 수 있었으나, `makePtr`가 더 안전합니다.)
* `DetectorParameters`는 `cv::makePtr<cv::aruco::DetectorParameters>()`로 생성합니다.
* `orderPoints` 함수 내 `std::min_element` 등의 반환값에서 인덱스를 얻기 위해 `std::distance`를 사용하도록 수정했습니다.
* `apriltagPoseEstimation` 함수 내 일부 변수명을 명확히 하고, 예외 처리 및 안정성을 위한 로직을 보강했습니다. `cv::normalize`를 시선 벡터 계산에 추가했습니다.
* `solvePnP`의 fallback으로 `SOLVEPNP_IPPE` 와 `SOLVEPNP_ITERATIVE`를 시도하도록 했습니다.
* `fixPoseAmbiguities` 함수에서 Z축 반전 외에 다른 축도 고려하여 오른손 좌표계를 유지하도록 수정하고, 유효성 검사를 추가했습니다.
* 마커 포즈를 시각화하기 위해 `cv::drawFrameAxes` 함수를 사용했습니다. (OpenCV 4.x 기준)
* 출력 텍스트에 마커 ID를 추가하고, 포맷을 일부 수정했습니다.

---

## 6. 코드 컴파일 ⚙️

터미널에서 저장된 C++ 소스 코드 파일이 있는 디렉토리로 이동한 후, 다음 명령어를 사용하여 컴파일합니다.
**C++17 표준**을 사용해야 합니다 (코드 내 structured binding `auto [R_opt, t_opt] = ...;` 사용 때문).

```bash
g++ -std=c++17 -o apriltag_pose_estimation apriltag_pose_estimation.cpp $(pkg-config --cflags --libs opencv4) -pthread
```
* `-pthread`는 C++ 표준 라이브러리의 스레딩 관련 기능을 사용할 경우 필요할 수 있습니다. (현재 코드에는 직접적인 멀티스레딩은 없지만, OpenCV 내부적으로 사용할 수 있으므로 추가하는 것이 안전할 수 있습니다.)
* 만약 `pkg-config --modversion opencv4`가 동작하지 않고 `opencv`로만 버전이 확인되었다면, 컴파일 명령어의 `opencv4` 부분을 `opencv`로 변경합니다.
    ```bash
    g++ -std=c++17 -o apriltag_pose_estimation apriltag_pose_estimation.cpp $(pkg-config --cflags --libs opencv) -pthread
    ```

만약 컴파일 중 `error: invalid initialization of reference of type ‘const cv::Ptr<cv::aruco::Dictionary>&’ from expression of type ‘cv::aruco::Dictionary’` 와 같은 오류가 발생하면, `main` 함수 내 `aruco_dict` 선언 부분을 확인하여 `cv::aruco::getPredefinedDictionary`의 반환 타입과 `cv::aruco::detectMarkers`가 요구하는 타입이 일치하도록 `cv::Ptr`로 올바르게 감쌌는지 확인합니다. (제공된 최종 코드는 이 부분이 수정되어 있습니다.)

---

## 7. 프로그램 실행 ▶️

컴파일이 성공적으로 완료되면, 실행 파일 (`apriltag_pose_estimation`)이 생성됩니다. 다음 명령어로 프로그램을 실행합니다.
```bash
./apriltag_pose_estimation
```
카메라가 연결되어 있고 프로그램이 정상적으로 실행되면, 카메라 영상 창이 나타나고 감지된 ArUco 마커 위에 좌표축과 함께 Roll, Pitch, Yaw, Translation (t) 벡터 정보가 표시됩니다.

* **종료**: 영상 창이 활성화된 상태에서 `q` 키를 누르면 프로그램이 종료됩니다.
* **GUI 환경**: Jetson이 데스크톱 환경(Ubuntu Desktop)에서 실행 중이어야 영상 창을 볼 수 있습니다. 헤드리스 환경이라면 코드 수정이 필요합니다 (예: 결과 저장 또는 네트워크 전송).
* **카메라 접근 권한**: 카메라 접근에 문제가 발생하면 `sudo ./apriltag_pose_estimation`과 같이 `sudo`를 사용하여 실행하거나, 현재 사용자를 `video` 그룹에 추가합니다 (`sudo usermod -aG video $USER`).

---

## 8. 추가 고려사항 및 디버깅 💡

* **카메라 소스**: 코드의 `cv::VideoCapture cap(0);` 부분은 첫 번째 연결된 카메라(/dev/video0)를 사용합니다. CSI 카메라나 다른 USB 카메라를 사용하려면 장치 번호를 변경하거나 GStreamer 파이프라인 문자열을 사용해야 할 수 있습니다.
    * 예시 (CSI 카메라): `cv::VideoCapture("nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)640, height=(int)480, format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink");`
* **성능 최적화**: Jetson 보드의 GPU 가속을 활용하기 위해 OpenCV가 CUDA 옵션과 함께 빌드되었는지 확인합니다. (JetPack 기본 OpenCV는 보통 CUDA 지원)
* **오류 메시지**: 실행 중 오류가 발생하면 터미널에 출력되는 메시지를 자세히 확인하여 문제의 원인(카메라 연결, OpenCV 설치, 라이브러리 링크, 코드 로직 등)을 파악하고 해결합니다.

```

