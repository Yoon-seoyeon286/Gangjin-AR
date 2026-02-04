#include "ar_tracker.h"
#include <iostream>

ARTracker::ARTracker() {
    slam = new SLAMSystem();

    // 모바일 카메라 파라미터
    fx = 500.0;
    fy = 500.0;
    cx = 320.0;
    cy = 240.0;

    std::cout << "[ARTracker] SLAM 기반 트래커 생성" << std::endl;
}

ARTracker::~ARTracker() {
    delete slam;
}

bool ARTracker::processFrame(const cv::Mat& frame) {
    return slam->processFrame(frame);
}

cv::Mat ARTracker::getViewMatrix() {
    return slam->getViewMatrix();
}

cv::Mat ARTracker::getProjectionMatrix(int width, int height) {
    double near = 0.01;
    double far = 1000.0;

    cv::Mat projMatrix = cv::Mat::zeros(4, 4, CV_64F);

    projMatrix.at<double>(0, 0) = 2.0 * fx / width;
    projMatrix.at<double>(1, 1) = 2.0 * fy / height;
    projMatrix.at<double>(0, 2) = 1.0 - 2.0 * cx / width;
    projMatrix.at<double>(1, 2) = 2.0 * cy / height - 1.0;
    projMatrix.at<double>(2, 2) = -(far + near) / (far - near);
    projMatrix.at<double>(2, 3) = -2.0 * far * near / (far - near);
    projMatrix.at<double>(3, 2) = -1.0;

    return projMatrix;
}

bool ARTracker::isInitialized() const {
    return slam->isInitialized();
}

bool ARTracker::isTracking() const {
    return slam->isTracking();
}

int ARTracker::getMapPointCount() const {
    return slam->getMapPointCount();
}

int ARTracker::getKeyFrameCount() const {
    return slam->getKeyFrameCount();
}
