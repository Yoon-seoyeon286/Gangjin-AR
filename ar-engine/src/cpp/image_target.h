#ifndef IMAGE_TARGET_H
#define IMAGE_TARGET_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

// 등록된 이미지 타겟
struct ImageTarget {
    int id;
    std::string name;
    cv::Mat image;                          // 원본 이미지 (그레이스케일)
    std::vector<cv::KeyPoint> keypoints;    // 특징점
    cv::Mat descriptors;                    // 디스크립터
    float widthMeters;                      // 실제 크기 (미터)
    float heightMeters;
};

// 감지된 타겟 결과
struct DetectedTarget {
    int targetId;
    std::string name;
    cv::Mat pose;                           // 4x4 변환 행렬
    std::vector<cv::Point2f> corners;       // 화면상 4개 코너
    float confidence;
    bool isTracking;
};

class ImageTargetTracker {
public:
    ImageTargetTracker();
    ~ImageTargetTracker();

    // 이미지 타겟 등록
    int addTarget(const cv::Mat& image, const std::string& name,
                  float widthMeters, float heightMeters = 0);

    // 이미지 타겟 등록 (이미지 데이터 직접)
    int addTargetFromData(const std::vector<uint8_t>& imageData,
                          int width, int height,
                          const std::string& name,
                          float widthMeters);

    // 프레임에서 타겟 감지
    bool detectTargets(const cv::Mat& frame, const cv::Mat& cameraMatrix);

    // 감지된 타겟 목록
    std::vector<DetectedTarget> getDetectedTargets() const;

    // 특정 타겟 제거
    void removeTarget(int targetId);

    // 모든 타겟 제거
    void clearTargets();

    // 등록된 타겟 수
    int getTargetCount() const { return targets.size(); }

private:
    // 특징점 추출
    void extractFeatures(const cv::Mat& image,
                        std::vector<cv::KeyPoint>& keypoints,
                        cv::Mat& descriptors);

    // 호모그래피로 포즈 계산
    bool computePose(const ImageTarget& target,
                     const std::vector<cv::Point2f>& srcPoints,
                     const std::vector<cv::Point2f>& dstPoints,
                     const cv::Mat& cameraMatrix,
                     cv::Mat& pose);

private:
    cv::Ptr<cv::ORB> orb;
    cv::Ptr<cv::BFMatcher> matcher;

    std::vector<ImageTarget> targets;
    std::vector<DetectedTarget> detectedTargets;

    int nextTargetId;

    // 설정
    const int MIN_MATCHES = 15;
    const float GOOD_MATCH_RATIO = 0.75f;
};

#endif
