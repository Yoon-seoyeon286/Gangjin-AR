#ifndef MAP_POINT_H
#define MAP_POINT_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>

// 3D 맵 포인트 - 월드 좌표계의 특징점
class MapPoint {
public:
    int id;
    cv::Point3f worldPos;           // 월드 좌표
    cv::Mat descriptor;              // ORB 디스크립터
    std::vector<int> observations;   // 이 포인트를 관측한 키프레임 ID들
    int matchCount;                  // 매칭 횟수
    bool isBad;                      // 유효하지 않은 포인트

    MapPoint(int _id, cv::Point3f pos, cv::Mat desc)
        : id(_id), worldPos(pos), descriptor(desc.clone()),
          matchCount(1), isBad(false) {}

    void addObservation(int keyframeId) {
        observations.push_back(keyframeId);
        matchCount++;
    }
};

// 키프레임 - 중요한 프레임 저장
class KeyFrame {
public:
    int id;
    cv::Mat image;                          // 그레이스케일 이미지
    cv::Mat pose;                           // 4x4 변환 행렬 (카메라 → 월드)
    std::vector<cv::KeyPoint> keypoints;    // 특징점들
    cv::Mat descriptors;                    // 디스크립터들
    std::vector<int> mapPointIds;           // 연결된 맵 포인트 ID (-1이면 없음)

    KeyFrame(int _id, const cv::Mat& img, const cv::Mat& _pose,
             const std::vector<cv::KeyPoint>& kps, const cv::Mat& descs)
        : id(_id), image(img.clone()), pose(_pose.clone()),
          keypoints(kps), descriptors(descs.clone()) {
        mapPointIds.resize(keypoints.size(), -1);
    }
};

#endif
