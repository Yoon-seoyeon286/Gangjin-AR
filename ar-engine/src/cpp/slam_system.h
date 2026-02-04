#ifndef SLAM_SYSTEM_H
#define SLAM_SYSTEM_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>
#include <map>
#include "map_point.h"
#include "plane_detector.h"
#include "image_target.h"

class SLAMSystem {
public:
    SLAMSystem();
    ~SLAMSystem();

    // 프레임 처리 - 메인 엔트리
    bool processFrame(const cv::Mat& frame);

    // 결과 반환
    cv::Mat getViewMatrix() const;
    cv::Mat getCameraPose() const;
    int getMapPointCount() const;
    int getKeyFrameCount() const;

    // 상태
    bool isInitialized() const { return initialized; }
    bool isTracking() const { return tracking; }

    // 평면 감지
    PlaneDetector* getPlaneDetector() { return planeDetector; }
    std::vector<DetectedPlane> getDetectedPlanes() const;

    // 이미지 타겟
    ImageTargetTracker* getImageTargetTracker() { return imageTargetTracker; }
    int addImageTarget(const cv::Mat& image, const std::string& name, float widthMeters);
    std::vector<DetectedTarget> getDetectedTargets() const;

private:
    // 초기화 (첫 두 프레임으로 맵 생성)
    bool initialize(const cv::Mat& frame);

    // 트래킹 (기존 맵으로 현재 위치 추정)
    bool track(const cv::Mat& frame);

    // 새 키프레임 필요 여부 판단
    bool needNewKeyFrame();

    // 키프레임 생성
    void createKeyFrame(const cv::Mat& frame);

    // 맵 포인트 삼각측량
    void triangulateNewPoints(KeyFrame& kf1, KeyFrame& kf2);

    // 루프 클로징 검사
    bool detectLoopClosure(const KeyFrame& currentKF);

    // 번들 조정 (로컬)
    void localBundleAdjustment();

    // 특징점 추출
    void extractFeatures(const cv::Mat& frame,
                        std::vector<cv::KeyPoint>& keypoints,
                        cv::Mat& descriptors);

    // 맵 포인트와 현재 프레임 매칭
    std::vector<cv::DMatch> matchWithMap(const cv::Mat& descriptors);

    // PnP로 포즈 추정
    bool estimatePosePnP(const std::vector<cv::Point3f>& worldPoints,
                         const std::vector<cv::Point2f>& imagePoints);

private:
    // ORB 특징점 검출기
    cv::Ptr<cv::ORB> orb;
    cv::Ptr<cv::BFMatcher> matcher;

    // 컴퓨터 비전 모듈
    PlaneDetector* planeDetector;
    ImageTargetTracker* imageTargetTracker;

    // 카메라 내부 파라미터
    cv::Mat K;              // 3x3 카메라 행렬
    cv::Mat distCoeffs;     // 왜곡 계수

    // 현재 카메라 포즈
    cv::Mat currentPose;    // 4x4 변환 행렬

    // 맵
    std::map<int, std::shared_ptr<MapPoint>> mapPoints;
    std::vector<std::shared_ptr<KeyFrame>> keyframes;

    // 현재 프레임 정보
    cv::Mat prevFrame;
    std::vector<cv::KeyPoint> prevKeypoints;
    cv::Mat prevDescriptors;

    // 상태
    bool initialized;
    bool tracking;
    int frameCount;
    int nextMapPointId;
    int nextKeyFrameId;

    // 설정
    const int MIN_INIT_MATCHES = 100;       // 초기화에 필요한 최소 매칭
    const int MIN_TRACKING_MATCHES = 20;    // 트래킹에 필요한 최소 매칭
    const int KEYFRAME_INTERVAL = 15;       // 키프레임 간격
    const float KEYFRAME_TRANSLATION = 0.1f; // 키프레임 생성 이동 거리
};

#endif
