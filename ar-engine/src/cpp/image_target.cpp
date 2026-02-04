#include "image_target.h"
#include <iostream>

ImageTargetTracker::ImageTargetTracker() {
    orb = cv::ORB::create(1000, 1.2f, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20);
    matcher = cv::BFMatcher::create(cv::NORM_HAMMING);
    nextTargetId = 0;
    std::cout << "[ImageTarget] 트래커 초기화" << std::endl;
}

ImageTargetTracker::~ImageTargetTracker() {
    clearTargets();
}

int ImageTargetTracker::addTarget(const cv::Mat& image, const std::string& name,
                                   float widthMeters, float heightMeters) {
    if (image.empty()) {
        std::cout << "[ImageTarget] 빈 이미지" << std::endl;
        return -1;
    }

    ImageTarget target;
    target.id = nextTargetId++;
    target.name = name;
    target.widthMeters = widthMeters;

    // 높이가 지정되지 않으면 비율로 계산
    if (heightMeters <= 0) {
        target.heightMeters = widthMeters * image.rows / image.cols;
    } else {
        target.heightMeters = heightMeters;
    }

    // 그레이스케일 변환
    if (image.channels() == 3) {
        cv::cvtColor(image, target.image, cv::COLOR_BGR2GRAY);
    } else if (image.channels() == 4) {
        cv::cvtColor(image, target.image, cv::COLOR_BGRA2GRAY);
    } else {
        target.image = image.clone();
    }

    // 특징점 추출
    extractFeatures(target.image, target.keypoints, target.descriptors);

    if (target.keypoints.size() < MIN_MATCHES) {
        std::cout << "[ImageTarget] 특징점 부족: " << target.keypoints.size() << std::endl;
        return -1;
    }

    targets.push_back(target);

    std::cout << "[ImageTarget] 타겟 등록 #" << target.id << " '" << name << "'"
              << " 특징점: " << target.keypoints.size()
              << " 크기: " << target.widthMeters << "x" << target.heightMeters << "m"
              << std::endl;

    return target.id;
}

int ImageTargetTracker::addTargetFromData(const std::vector<uint8_t>& imageData,
                                           int width, int height,
                                           const std::string& name,
                                           float widthMeters) {
    if (imageData.empty() || width <= 0 || height <= 0) {
        return -1;
    }

    // RGBA 데이터를 Mat으로 변환
    cv::Mat rgba(height, width, CV_8UC4, const_cast<uint8_t*>(imageData.data()));
    cv::Mat gray;
    cv::cvtColor(rgba, gray, cv::COLOR_RGBA2GRAY);

    return addTarget(gray, name, widthMeters);
}

void ImageTargetTracker::extractFeatures(const cv::Mat& image,
                                          std::vector<cv::KeyPoint>& keypoints,
                                          cv::Mat& descriptors) {
    orb->detectAndCompute(image, cv::noArray(), keypoints, descriptors);
}

bool ImageTargetTracker::detectTargets(const cv::Mat& frame, const cv::Mat& cameraMatrix) {
    detectedTargets.clear();

    if (targets.empty() || frame.empty()) {
        return false;
    }

    // 프레임 특징점 추출
    std::vector<cv::KeyPoint> frameKeypoints;
    cv::Mat frameDescriptors;
    extractFeatures(frame, frameKeypoints, frameDescriptors);

    if (frameKeypoints.size() < MIN_MATCHES) {
        return false;
    }

    // 각 타겟에 대해 매칭 시도
    for (const auto& target : targets) {
        if (target.descriptors.empty()) continue;

        // KNN 매칭
        std::vector<std::vector<cv::DMatch>> knnMatches;
        matcher->knnMatch(target.descriptors, frameDescriptors, knnMatches, 2);

        // Lowe's ratio test
        std::vector<cv::DMatch> goodMatches;
        for (const auto& knn : knnMatches) {
            if (knn.size() >= 2 && knn[0].distance < GOOD_MATCH_RATIO * knn[1].distance) {
                goodMatches.push_back(knn[0]);
            }
        }

        if (goodMatches.size() < MIN_MATCHES) {
            continue;
        }

        // 매칭된 점들 추출
        std::vector<cv::Point2f> srcPoints, dstPoints;
        for (const auto& m : goodMatches) {
            srcPoints.push_back(target.keypoints[m.queryIdx].pt);
            dstPoints.push_back(frameKeypoints[m.trainIdx].pt);
        }

        // 호모그래피 계산
        cv::Mat mask;
        cv::Mat H = cv::findHomography(srcPoints, dstPoints, cv::RANSAC, 5.0, mask);

        if (H.empty()) {
            continue;
        }

        // 인라이어 카운트
        int inliers = cv::countNonZero(mask);
        if (inliers < MIN_MATCHES) {
            continue;
        }

        // 타겟 코너 투영
        std::vector<cv::Point2f> targetCorners = {
            cv::Point2f(0, 0),
            cv::Point2f(target.image.cols, 0),
            cv::Point2f(target.image.cols, target.image.rows),
            cv::Point2f(0, target.image.rows)
        };

        std::vector<cv::Point2f> projectedCorners;
        cv::perspectiveTransform(targetCorners, projectedCorners, H);

        // 유효한 사각형인지 확인 (볼록 다각형)
        if (!cv::isContourConvex(projectedCorners)) {
            continue;
        }

        // 포즈 계산
        DetectedTarget detected;
        detected.targetId = target.id;
        detected.name = target.name;
        detected.corners = projectedCorners;
        detected.confidence = static_cast<float>(inliers) / goodMatches.size();
        detected.isTracking = true;

        if (computePose(target, srcPoints, dstPoints, cameraMatrix, detected.pose)) {
            detectedTargets.push_back(detected);

            std::cout << "[ImageTarget] 감지: '" << target.name << "'"
                      << " 신뢰도: " << (detected.confidence * 100) << "%"
                      << std::endl;
        }
    }

    return !detectedTargets.empty();
}

bool ImageTargetTracker::computePose(const ImageTarget& target,
                                      const std::vector<cv::Point2f>& srcPoints,
                                      const std::vector<cv::Point2f>& dstPoints,
                                      const cv::Mat& cameraMatrix,
                                      cv::Mat& pose) {
    // 타겟 좌표를 3D로 변환 (Z=0 평면)
    std::vector<cv::Point3f> objectPoints;
    for (const auto& pt : srcPoints) {
        float x = (pt.x / target.image.cols - 0.5f) * target.widthMeters;
        float y = (pt.y / target.image.rows - 0.5f) * target.heightMeters;
        objectPoints.push_back(cv::Point3f(x, y, 0));
    }

    cv::Mat rvec, tvec;
    cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64F);

    bool success = cv::solvePnPRansac(objectPoints, dstPoints, cameraMatrix, distCoeffs,
                                       rvec, tvec, false, 100, 8.0, 0.99);

    if (!success) {
        return false;
    }

    // 회전 벡터 → 회전 행렬
    cv::Mat R;
    cv::Rodrigues(rvec, R);

    // 4x4 포즈 행렬 생성
    pose = cv::Mat::eye(4, 4, CV_64F);
    R.copyTo(pose(cv::Rect(0, 0, 3, 3)));
    tvec.copyTo(pose(cv::Rect(3, 0, 1, 3)));

    return true;
}

std::vector<DetectedTarget> ImageTargetTracker::getDetectedTargets() const {
    return detectedTargets;
}

void ImageTargetTracker::removeTarget(int targetId) {
    auto it = std::remove_if(targets.begin(), targets.end(),
                              [targetId](const ImageTarget& t) { return t.id == targetId; });
    targets.erase(it, targets.end());
}

void ImageTargetTracker::clearTargets() {
    targets.clear();
    detectedTargets.clear();
    nextTargetId = 0;
}
