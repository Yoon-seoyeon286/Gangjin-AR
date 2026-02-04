#include "slam_system.h"
#include "plane_detector.h"
#include "image_target.h"
#include <iostream>

SLAMSystem::SLAMSystem() {
    // 컴퓨터 비전 모듈 초기화
    planeDetector = new PlaneDetector();
    imageTargetTracker = new ImageTargetTracker();
    // ORB 특징점 검출기 (더 많은 특징점)
    orb = cv::ORB::create(2000, 1.2f, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20);
    matcher = cv::BFMatcher::create(cv::NORM_HAMMING, true);

    // 카메라 파라미터 (모바일 기준)
    float fx = 500.0f;
    float fy = 500.0f;
    float cx = 320.0f;
    float cy = 240.0f;

    K = (cv::Mat_<double>(3, 3) <<
        fx, 0, cx,
        0, fy, cy,
        0, 0, 1);

    distCoeffs = cv::Mat::zeros(4, 1, CV_64F);

    // 초기 포즈 (단위 행렬)
    currentPose = cv::Mat::eye(4, 4, CV_64F);

    initialized = false;
    tracking = false;
    frameCount = 0;
    nextMapPointId = 0;
    nextKeyFrameId = 0;

    std::cout << "[SLAM] 시스템 초기화 완료" << std::endl;
}

SLAMSystem::~SLAMSystem() {
    mapPoints.clear();
    keyframes.clear();
    delete planeDetector;
    delete imageTargetTracker;
}

bool SLAMSystem::processFrame(const cv::Mat& frame) {
    frameCount++;

    if (!initialized) {
        return initialize(frame);
    } else {
        return track(frame);
    }
}

bool SLAMSystem::initialize(const cv::Mat& frame) {
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    extractFeatures(frame, keypoints, descriptors);

    if (keypoints.size() < MIN_INIT_MATCHES) {
        std::cout << "[SLAM] 초기화 실패: 특징점 부족 (" << keypoints.size() << ")" << std::endl;
        return false;
    }

    if (prevFrame.empty()) {
        // 첫 프레임 저장
        prevFrame = frame.clone();
        prevKeypoints = keypoints;
        prevDescriptors = descriptors.clone();
        std::cout << "[SLAM] 첫 프레임 저장 (" << keypoints.size() << " 특징점)" << std::endl;
        return true;
    }

    // 두 번째 프레임 - 매칭 시도
    std::vector<cv::DMatch> matches;
    if (!prevDescriptors.empty() && !descriptors.empty()) {
        matcher->match(prevDescriptors, descriptors, matches);
    }

    if (matches.size() < MIN_INIT_MATCHES) {
        std::cout << "[SLAM] 초기화 실패: 매칭 부족 (" << matches.size() << ")" << std::endl;
        prevFrame = frame.clone();
        prevKeypoints = keypoints;
        prevDescriptors = descriptors.clone();
        return false;
    }

    // 좋은 매칭 필터링
    std::vector<cv::DMatch> goodMatches;
    double minDist = 100;
    for (const auto& m : matches) {
        if (m.distance < minDist) minDist = m.distance;
    }
    for (const auto& m : matches) {
        if (m.distance <= std::max(2.0 * minDist, 30.0)) {
            goodMatches.push_back(m);
        }
    }

    if (goodMatches.size() < MIN_INIT_MATCHES / 2) {
        std::cout << "[SLAM] 초기화 실패: 좋은 매칭 부족 (" << goodMatches.size() << ")" << std::endl;
        return false;
    }

    // Essential Matrix 계산
    std::vector<cv::Point2f> pts1, pts2;
    for (const auto& m : goodMatches) {
        pts1.push_back(prevKeypoints[m.queryIdx].pt);
        pts2.push_back(keypoints[m.trainIdx].pt);
    }

    cv::Mat mask;
    cv::Mat E = cv::findEssentialMat(pts1, pts2, K, cv::RANSAC, 0.999, 1.0, mask);

    if (E.empty()) {
        std::cout << "[SLAM] 초기화 실패: Essential Matrix 계산 실패" << std::endl;
        return false;
    }

    // 포즈 복원
    cv::Mat R, t;
    int inliers = cv::recoverPose(E, pts1, pts2, K, R, t, mask);

    if (inliers < 30) {
        std::cout << "[SLAM] 초기화 실패: 인라이어 부족 (" << inliers << ")" << std::endl;
        return false;
    }

    // 첫 키프레임 생성 (원점)
    cv::Mat pose1 = cv::Mat::eye(4, 4, CV_64F);
    auto kf1 = std::make_shared<KeyFrame>(nextKeyFrameId++, prevFrame, pose1, prevKeypoints, prevDescriptors);
    keyframes.push_back(kf1);

    // 두 번째 키프레임 생성
    cv::Mat pose2 = cv::Mat::eye(4, 4, CV_64F);
    R.copyTo(pose2(cv::Rect(0, 0, 3, 3)));
    t.copyTo(pose2(cv::Rect(3, 0, 1, 3)));
    auto kf2 = std::make_shared<KeyFrame>(nextKeyFrameId++, frame, pose2, keypoints, descriptors);
    keyframes.push_back(kf2);

    // 삼각측량으로 맵 포인트 생성
    triangulateNewPoints(*kf1, *kf2);

    currentPose = pose2.clone();
    prevFrame = frame.clone();
    prevKeypoints = keypoints;
    prevDescriptors = descriptors.clone();

    initialized = true;
    tracking = true;

    std::cout << "[SLAM] 초기화 성공! 맵포인트: " << mapPoints.size()
              << ", 키프레임: " << keyframes.size() << std::endl;

    return true;
}

bool SLAMSystem::track(const cv::Mat& frame) {
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    extractFeatures(frame, keypoints, descriptors);

    if (keypoints.size() < MIN_TRACKING_MATCHES) {
        tracking = false;
        std::cout << "[SLAM] 트래킹 실패: 특징점 부족" << std::endl;
        return false;
    }

    // 맵 포인트와 매칭
    std::vector<cv::DMatch> matches = matchWithMap(descriptors);

    if (matches.size() < MIN_TRACKING_MATCHES) {
        // 맵 매칭 실패 시 이전 프레임과 매칭
        std::vector<cv::DMatch> frameMatches;
        if (!prevDescriptors.empty()) {
            matcher->match(prevDescriptors, descriptors, frameMatches);
        }

        if (frameMatches.size() < MIN_TRACKING_MATCHES) {
            tracking = false;
            std::cout << "[SLAM] 트래킹 실패: 매칭 부족" << std::endl;
            return false;
        }

        // Essential Matrix로 상대 포즈 계산
        std::vector<cv::Point2f> pts1, pts2;
        for (const auto& m : frameMatches) {
            pts1.push_back(prevKeypoints[m.queryIdx].pt);
            pts2.push_back(keypoints[m.trainIdx].pt);
        }

        cv::Mat mask;
        cv::Mat E = cv::findEssentialMat(pts1, pts2, K, cv::RANSAC, 0.999, 1.0, mask);

        if (!E.empty()) {
            cv::Mat R, t;
            cv::recoverPose(E, pts1, pts2, K, R, t, mask);

            // 포즈 업데이트 (누적)
            cv::Mat deltaPose = cv::Mat::eye(4, 4, CV_64F);
            R.copyTo(deltaPose(cv::Rect(0, 0, 3, 3)));
            t.copyTo(deltaPose(cv::Rect(3, 0, 1, 3)));

            currentPose = currentPose * deltaPose;
            tracking = true;
        }
    } else {
        // PnP로 포즈 추정
        std::vector<cv::Point3f> worldPoints;
        std::vector<cv::Point2f> imagePoints;

        for (const auto& m : matches) {
            auto it = mapPoints.find(m.queryIdx);
            if (it != mapPoints.end() && !it->second->isBad) {
                worldPoints.push_back(it->second->worldPos);
                imagePoints.push_back(keypoints[m.trainIdx].pt);
            }
        }

        if (worldPoints.size() >= 6) {
            tracking = estimatePosePnP(worldPoints, imagePoints);
        }
    }

    // 키프레임 필요 여부 확인
    if (tracking && needNewKeyFrame()) {
        createKeyFrame(frame);
    }

    // === 컴퓨터 비전 처리 ===

    // 평면 감지 (맵 포인트가 충분할 때)
    if (mapPoints.size() >= 50 && frameCount % 30 == 0) {
        std::vector<cv::Point3f> pts;
        for (const auto& pair : mapPoints) {
            if (!pair.second->isBad) {
                pts.push_back(pair.second->worldPos);
            }
        }
        planeDetector->detectPlanes(frame, pts, currentPose);
    }

    // 이미지 타겟 감지
    if (imageTargetTracker->getTargetCount() > 0 && frameCount % 5 == 0) {
        imageTargetTracker->detectTargets(frame, K);
    }

    prevFrame = frame.clone();
    prevKeypoints = keypoints;
    prevDescriptors = descriptors.clone();

    return tracking;
}

void SLAMSystem::extractFeatures(const cv::Mat& frame,
                                  std::vector<cv::KeyPoint>& keypoints,
                                  cv::Mat& descriptors) {
    orb->detectAndCompute(frame, cv::noArray(), keypoints, descriptors);
}

std::vector<cv::DMatch> SLAMSystem::matchWithMap(const cv::Mat& descriptors) {
    std::vector<cv::DMatch> matches;

    if (mapPoints.empty() || descriptors.empty()) {
        return matches;
    }

    // 모든 맵 포인트의 디스크립터 수집
    cv::Mat mapDescriptors;
    std::vector<int> mapPointIdxs;

    for (const auto& pair : mapPoints) {
        if (!pair.second->isBad) {
            mapDescriptors.push_back(pair.second->descriptor);
            mapPointIdxs.push_back(pair.first);
        }
    }

    if (mapDescriptors.empty()) {
        return matches;
    }

    std::vector<cv::DMatch> rawMatches;
    matcher->match(mapDescriptors, descriptors, rawMatches);

    // 좋은 매칭 필터링
    for (const auto& m : rawMatches) {
        if (m.distance < 50) {
            cv::DMatch goodMatch;
            goodMatch.queryIdx = mapPointIdxs[m.queryIdx];
            goodMatch.trainIdx = m.trainIdx;
            goodMatch.distance = m.distance;
            matches.push_back(goodMatch);
        }
    }

    return matches;
}

bool SLAMSystem::estimatePosePnP(const std::vector<cv::Point3f>& worldPoints,
                                   const std::vector<cv::Point2f>& imagePoints) {
    if (worldPoints.size() < 6) return false;

    cv::Mat rvec, tvec;
    bool success = cv::solvePnPRansac(worldPoints, imagePoints, K, distCoeffs,
                                       rvec, tvec, false, 100, 8.0, 0.99);

    if (success) {
        cv::Mat R;
        cv::Rodrigues(rvec, R);

        currentPose = cv::Mat::eye(4, 4, CV_64F);
        R.copyTo(currentPose(cv::Rect(0, 0, 3, 3)));
        tvec.copyTo(currentPose(cv::Rect(3, 0, 1, 3)));
    }

    return success;
}

bool SLAMSystem::needNewKeyFrame() {
    if (keyframes.empty()) return true;

    // 프레임 간격 체크
    if (frameCount % KEYFRAME_INTERVAL != 0) return false;

    // 이동 거리 체크
    cv::Mat lastPose = keyframes.back()->pose;
    cv::Mat t1 = lastPose(cv::Rect(3, 0, 1, 3));
    cv::Mat t2 = currentPose(cv::Rect(3, 0, 1, 3));
    double dist = cv::norm(t2 - t1);

    return dist > KEYFRAME_TRANSLATION;
}

void SLAMSystem::createKeyFrame(const cv::Mat& frame) {
    auto kf = std::make_shared<KeyFrame>(nextKeyFrameId++, frame, currentPose,
                                          prevKeypoints, prevDescriptors);
    keyframes.push_back(kf);

    // 이전 키프레임과 삼각측량
    if (keyframes.size() >= 2) {
        triangulateNewPoints(*keyframes[keyframes.size() - 2], *kf);
    }

    // 루프 클로징 검사
    if (keyframes.size() > 10) {
        detectLoopClosure(*kf);
    }

    std::cout << "[SLAM] 키프레임 생성 #" << kf->id
              << " (맵포인트: " << mapPoints.size() << ")" << std::endl;
}

void SLAMSystem::triangulateNewPoints(KeyFrame& kf1, KeyFrame& kf2) {
    // 두 키프레임 간 매칭
    std::vector<cv::DMatch> matches;
    if (!kf1.descriptors.empty() && !kf2.descriptors.empty()) {
        matcher->match(kf1.descriptors, kf2.descriptors, matches);
    }

    // Projection 행렬
    cv::Mat P1 = K * kf1.pose(cv::Rect(0, 0, 4, 3));
    cv::Mat P2 = K * kf2.pose(cv::Rect(0, 0, 4, 3));

    for (const auto& m : matches) {
        if (m.distance > 50) continue;

        // 이미 맵 포인트가 있으면 스킵
        if (kf1.mapPointIds[m.queryIdx] >= 0) continue;

        cv::Point2f pt1 = kf1.keypoints[m.queryIdx].pt;
        cv::Point2f pt2 = kf2.keypoints[m.trainIdx].pt;

        // 삼각측량
        cv::Mat pts4D;
        std::vector<cv::Point2f> pts1Vec = {pt1};
        std::vector<cv::Point2f> pts2Vec = {pt2};

        cv::triangulatePoints(P1, P2, pts1Vec, pts2Vec, pts4D);

        // 동차 좌표 → 3D
        float w = pts4D.at<float>(3, 0);
        if (std::abs(w) < 1e-6) continue;

        cv::Point3f worldPt(
            pts4D.at<float>(0, 0) / w,
            pts4D.at<float>(1, 0) / w,
            pts4D.at<float>(2, 0) / w
        );

        // 카메라 앞에 있는지 체크
        if (worldPt.z < 0) continue;

        // 맵 포인트 생성
        auto mp = std::make_shared<MapPoint>(nextMapPointId++, worldPt, kf1.descriptors.row(m.queryIdx));
        mp->addObservation(kf1.id);
        mp->addObservation(kf2.id);

        mapPoints[mp->id] = mp;
        kf1.mapPointIds[m.queryIdx] = mp->id;
        kf2.mapPointIds[m.trainIdx] = mp->id;
    }
}

bool SLAMSystem::detectLoopClosure(const KeyFrame& currentKF) {
    // 간단한 BoW 기반 루프 클로징
    // 현재는 디스크립터 매칭으로 유사 키프레임 검색

    int bestMatch = -1;
    int bestMatchCount = 0;

    for (size_t i = 0; i < keyframes.size() - 5; i++) {
        const auto& kf = keyframes[i];

        std::vector<cv::DMatch> matches;
        if (!kf->descriptors.empty() && !currentKF.descriptors.empty()) {
            matcher->match(kf->descriptors, currentKF.descriptors, matches);
        }

        // 좋은 매칭 카운트
        int goodCount = 0;
        for (const auto& m : matches) {
            if (m.distance < 40) goodCount++;
        }

        if (goodCount > bestMatchCount && goodCount > 50) {
            bestMatchCount = goodCount;
            bestMatch = i;
        }
    }

    if (bestMatch >= 0) {
        std::cout << "[SLAM] 루프 클로징 감지! KF " << currentKF.id
                  << " ↔ KF " << keyframes[bestMatch]->id << std::endl;

        // TODO: 포즈 그래프 최적화
        localBundleAdjustment();
        return true;
    }

    return false;
}

void SLAMSystem::localBundleAdjustment() {
    // 간단한 로컬 BA (현재는 placeholder)
    // 실제 구현은 g2o 또는 Ceres 필요
    std::cout << "[SLAM] 로컬 번들 조정 실행" << std::endl;
}

cv::Mat SLAMSystem::getViewMatrix() const {
    // View Matrix = 카메라 포즈의 역행렬
    return currentPose.inv();
}

cv::Mat SLAMSystem::getCameraPose() const {
    return currentPose.clone();
}

int SLAMSystem::getMapPointCount() const {
    return static_cast<int>(mapPoints.size());
}

int SLAMSystem::getKeyFrameCount() const {
    return static_cast<int>(keyframes.size());
}

// ==================== 컴퓨터 비전 ====================

std::vector<DetectedPlane> SLAMSystem::getDetectedPlanes() const {
    return planeDetector->getDetectedPlanes();
}

int SLAMSystem::addImageTarget(const cv::Mat& image, const std::string& name, float widthMeters) {
    return imageTargetTracker->addTarget(image, name, widthMeters);
}

std::vector<DetectedTarget> SLAMSystem::getDetectedTargets() const {
    return imageTargetTracker->getDetectedTargets();
}
