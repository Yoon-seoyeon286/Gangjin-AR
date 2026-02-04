#ifndef AR_TRACKER_H
#define AR_TRACKER_H

#include <opencv2/opencv.hpp>
#include "slam_system.h"

class ARTracker {
public:
    ARTracker();
    ~ARTracker();

    // 새로운 프레임 처리
    bool processFrame(const cv::Mat& frame);

    // Three.js에서 사용할 행렬 가져오기
    cv::Mat getViewMatrix();
    cv::Mat getProjectionMatrix(int width, int height);

    // SLAM 상태 정보
    bool isInitialized() const;
    bool isTracking() const;
    int getMapPointCount() const;
    int getKeyFrameCount() const;

    // SLAM 시스템 접근 (컴퓨터 비전용)
    SLAMSystem* getSLAM() { return slam; }

private:
    SLAMSystem* slam;

    // 카메라 파라미터
    double fx, fy, cx, cy;
};

#endif
