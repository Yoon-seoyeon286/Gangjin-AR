#include "pose_estimator.h"
#include <iostream>

PoseEstimator::PoseEstimator() {
    // 웹캠 기준 카메라 파라미터
    focal_length = 800.0;
    
    K = (cv::Mat_<double>(3, 3) << 
        focal_length, 0, 320,
        0, focal_length, 240,
        0, 0, 1
    );
    
    R = cv::Mat::eye(3, 3, CV_64F);
    t = cv::Mat::zeros(3, 1, CV_64F);
}

bool PoseEstimator::estimatePose(
    const std::vector<cv::Point2f>& points1,
    const std::vector<cv::Point2f>& points2
) {
    if (points1.size() < 8 || points2.size() < 8) {
        return false;
    }
    
    cv::Mat mask;
    cv::Mat E = cv::findEssentialMat(
        points1, 
        points2, 
        K,
        cv::RANSAC,
        0.999,
        1.0,
        mask
    );
    
    if (E.empty()) {
        return false;
    }
    
    cv::Mat R_new, t_new;
    int inliers = cv::recoverPose(
        E,
        points1,
        points2,
        K,
        R_new,
        t_new,
        mask
    );
    
    if (inliers < 20) {
        return false;
    }
    
    // 포즈 누적
    R = R_new * R;
    t = t + R * t_new;
    
    return true;
}

cv::Mat PoseEstimator::getRotationMatrix() const {
    return R.clone();
}

cv::Mat PoseEstimator::getTranslationVector() const {
    return t.clone();
}

cv::Mat PoseEstimator::getCameraMatrix() const {
    return K.clone();
}