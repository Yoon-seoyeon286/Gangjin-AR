#ifndef POSE_ESTIMATOR_H
#define POSE_ESTIMATOR_H

#include <opencv2/opencv.hpp>
#include <vector>

class PoseEstimator {
public:
    PoseEstimator();
    
    // 매칭된 점들로부터 카메라 포즈 추정
    bool estimatePose(
        const std::vector<cv::Point2f>& points1,
        const std::vector<cv::Point2f>& points2
    );
    
    // 현재 카메라 포즈 정보 가져오기
    cv::Mat getRotationMatrix() const;        // 회전 행렬
    cv::Mat getTranslationVector() const;     // 이동 벡터
    cv::Mat getCameraMatrix() const;          // 카메라 내부 행렬
    
private:
    cv::Mat K;              // 카메라 내부 행렬 (Camera Intrinsic)
    cv::Mat R;              // 누적 회전 행렬
    cv::Mat t;              // 누적 이동 벡터
    double focal_length;    // 초점 거리
};

#endif