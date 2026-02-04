#ifndef PLANE_DETECTOR_H
#define PLANE_DETECTOR_H

#include <opencv2/opencv.hpp>
#include <vector>

// 감지된 평면 정보
struct DetectedPlane {
    int id;
    cv::Point3f center;         // 평면 중심 (월드 좌표)
    cv::Point3f normal;         // 평면 법선 벡터
    float width;                // 평면 너비 (미터)
    float height;               // 평면 높이 (미터)
    std::vector<cv::Point3f> corners;  // 4개 코너 (월드 좌표)
    bool isHorizontal;          // 수평면 여부
    float confidence;           // 신뢰도 (0~1)
};

class PlaneDetector {
public:
    PlaneDetector();
    ~PlaneDetector();

    // 평면 감지 실행
    bool detectPlanes(const cv::Mat& frame,
                      const std::vector<cv::Point3f>& mapPoints,
                      const cv::Mat& cameraPose);

    // 감지된 평면 반환
    std::vector<DetectedPlane> getDetectedPlanes() const;

    // 특정 2D 좌표가 평면 위인지 확인
    bool hitTest(float screenX, float screenY,
                 const cv::Mat& cameraMatrix,
                 const cv::Mat& cameraPose,
                 cv::Point3f& hitPoint, int& planeId);

private:
    // RANSAC으로 평면 피팅
    bool fitPlaneRANSAC(const std::vector<cv::Point3f>& points,
                        cv::Point3f& planeNormal, float& planeD,
                        std::vector<int>& inliers);

    // 평면이 수평인지 확인
    bool isPlaneHorizontal(const cv::Point3f& normal);

    // 평면 경계 계산
    void computePlaneBounds(const std::vector<cv::Point3f>& inlierPoints,
                            const cv::Point3f& normal,
                            DetectedPlane& plane);

    // 기존 평면과 병합 가능한지 확인
    int findMergeablePlane(const DetectedPlane& newPlane);

    // 평면 병합
    void mergePlanes(int existingIdx, const DetectedPlane& newPlane);

private:
    std::vector<DetectedPlane> planes;
    int nextPlaneId;

    // 설정
    const float RANSAC_THRESHOLD = 0.02f;      // 2cm 허용 오차
    const int RANSAC_ITERATIONS = 100;
    const int MIN_PLANE_POINTS = 50;
    const float HORIZONTAL_THRESHOLD = 0.9f;   // cos(25도)
    const float MERGE_DISTANCE = 0.1f;         // 10cm 이내면 병합
};

#endif
