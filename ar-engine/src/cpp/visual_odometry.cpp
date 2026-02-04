#include "visual_odometry.h"
#include <chrono>
#include <algorithm>
#include <cmath>

// ============================================================================
// 생성자 / 소멸자
// ============================================================================

VisualOdometry::VisualOdometry() {
    Config defaultConfig;
    setConfig(defaultConfig);
    reset();
}

VisualOdometry::VisualOdometry(const Config& config) {
    setConfig(config);
    reset();
}

VisualOdometry::~VisualOdometry() {
    // 스마트 포인터 사용으로 자동 정리
}

// ============================================================================
// 설정
// ============================================================================

void VisualOdometry::setConfig(const Config& cfg) {
    config = cfg;

    // 카메라 행렬 설정
    K = (cv::Mat_<double>(3, 3) <<
        config.focalLength, 0, config.cx,
        0, config.focalLength, config.cy,
        0, 0, 1);

    distCoeffs = cv::Mat::zeros(4, 1, CV_64F);

    // FAST 검출기 생성
    fastDetector = cv::FastFeatureDetector::create(
        config.fastThreshold,
        config.fastNonmaxSuppression
    );
}

void VisualOdometry::setCameraParams(float fx, float fy, float cx, float cy) {
    config.focalLength = fx;  // fy는 보통 fx와 같다고 가정
    config.cx = cx;
    config.cy = cy;

    K = (cv::Mat_<double>(3, 3) <<
        fx, 0, cx,
        0, fy, cy,
        0, 0, 1);
}

void VisualOdometry::reset() {
    prevGray.release();
    currGray.release();
    prevPoints.clear();
    currPoints.clear();
    pointIds.clear();
    pointAges.clear();

    R_total = cv::Mat::eye(3, 3, CV_64F);
    t_total = cv::Mat::zeros(3, 1, CV_64F);
    scale = 1.0;

    frameCount = 0;
    nextPointId = 0;
    initialized = false;

    frameData = FrameData();
}

// ============================================================================
// 메인 처리 함수
// ============================================================================

bool VisualOdometry::processFrame(const cv::Mat& gray) {
    auto startTime = std::chrono::high_resolution_clock::now();

    // 입력 검증
    if (gray.empty() || gray.type() != CV_8UC1) {
        return false;
    }

    // 현재 프레임 저장
    currGray = gray.clone();
    frameCount++;

    if (!initialized) {
        // 첫 프레임: 특징점 추출만
        extractFeatures(currGray);

        if (currPoints.size() >= (size_t)config.minInliers) {
            initialized = true;
            frameData.initialized = true;
        }

        // 다음 프레임 준비
        prevGray = currGray.clone();
        prevPoints = currPoints;

    } else {
        // 이후 프레임: 특징점 추적 + 포즈 추정
        trackFeatures(currGray);

        if (currPoints.size() >= (size_t)config.minInliers) {
            bool poseOk = estimatePose();
            frameData.tracking = poseOk;

            if (poseOk) {
                updateViewMatrix();
            }
        } else {
            frameData.tracking = false;
        }

        // 특징점이 부족하면 새로 추출
        if (currPoints.size() < (size_t)(config.maxFeatures / 2)) {
            extractFeatures(currGray);
        }

        // 다음 프레임 준비
        prevGray = currGray.clone();
        prevPoints = currPoints;
    }

    // FrameData 업데이트 (JavaScript 전송용)
    updateFrameData();

    // 처리 시간 계산
    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
    frameData.processingTimeMs = duration.count() / 1000.0f;
    frameData.frameNumber = frameCount;

    return true;
}

bool VisualOdometry::processFrameRGBA(int width, int height, const uint8_t* data) {
    // RGBA → Gray 변환
    cv::Mat rgba(height, width, CV_8UC4, const_cast<uint8_t*>(data));
    cv::Mat gray;
    cv::cvtColor(rgba, gray, cv::COLOR_RGBA2GRAY);

    // 카메라 파라미터 자동 조정 (첫 프레임에서)
    if (frameCount == 0) {
        config.cx = width / 2.0f;
        config.cy = height / 2.0f;
        // 초점 거리 추정 (FOV ~60도 가정)
        config.focalLength = width / (2.0f * std::tan(30.0f * M_PI / 180.0f));

        K = (cv::Mat_<double>(3, 3) <<
            config.focalLength, 0, config.cx,
            0, config.focalLength, config.cy,
            0, 0, 1);
    }

    return processFrame(gray);
}

// ============================================================================
// 특징점 추출 (FAST)
// ============================================================================

void VisualOdometry::extractFeatures(const cv::Mat& gray) {
    std::vector<cv::KeyPoint> keypoints;

    // FAST 특징점 검출
    fastDetector->detect(gray, keypoints);

    // 응답 강도로 정렬 (강한 것 우선)
    std::sort(keypoints.begin(), keypoints.end(),
        [](const cv::KeyPoint& a, const cv::KeyPoint& b) {
            return a.response > b.response;
        });

    // 최대 개수 제한
    if (keypoints.size() > (size_t)config.maxFeatures) {
        keypoints.resize(config.maxFeatures);
    }

    // 기존 추적 중인 특징점과 합치기
    // 새 특징점은 기존 특징점과 최소 거리 유지
    const float minDist = 10.0f;
    std::vector<cv::Point2f> newPoints;
    std::vector<int> newIds;
    std::vector<int> newAges;

    // 기존 특징점 유지
    for (size_t i = 0; i < currPoints.size(); i++) {
        newPoints.push_back(currPoints[i]);
        newIds.push_back(pointIds[i]);
        newAges.push_back(pointAges[i]);
    }

    // 새 특징점 추가 (기존 특징점과 거리 확인)
    for (const auto& kp : keypoints) {
        bool tooClose = false;

        for (const auto& existingPt : newPoints) {
            float dx = kp.pt.x - existingPt.x;
            float dy = kp.pt.y - existingPt.y;
            if (dx * dx + dy * dy < minDist * minDist) {
                tooClose = true;
                break;
            }
        }

        if (!tooClose && newPoints.size() < (size_t)config.maxFeatures) {
            newPoints.push_back(kp.pt);
            newIds.push_back(nextPointId++);
            newAges.push_back(0);
        }
    }

    currPoints = newPoints;
    pointIds = newIds;
    pointAges = newAges;
}

// ============================================================================
// 특징점 추적 (Optical Flow)
// ============================================================================

void VisualOdometry::trackFeatures(const cv::Mat& gray) {
    if (prevPoints.empty() || prevGray.empty()) {
        return;
    }

    std::vector<cv::Point2f> nextPoints;
    std::vector<uchar> status;
    std::vector<float> err;

    // Lucas-Kanade Optical Flow
    cv::Size winSize(config.lkWinSize, config.lkWinSize);
    cv::TermCriteria criteria(
        cv::TermCriteria::COUNT | cv::TermCriteria::EPS,
        config.lkMaxIter,
        config.lkEpsilon
    );

    cv::calcOpticalFlowPyrLK(
        prevGray, gray,
        prevPoints, nextPoints,
        status, err,
        winSize,
        config.lkMaxLevel,
        criteria
    );

    // 추적 성공한 특징점만 유지
    std::vector<cv::Point2f> goodPrev;
    std::vector<cv::Point2f> goodCurr;
    std::vector<int> goodIds;
    std::vector<int> goodAges;

    // Flow 벡터 저장 (JavaScript 전송용)
    frameData.flowVectors.clear();
    frameData.matches.clear();

    for (size_t i = 0; i < status.size(); i++) {
        if (status[i]) {
            // 이미지 경계 체크
            if (nextPoints[i].x >= 0 && nextPoints[i].x < gray.cols &&
                nextPoints[i].y >= 0 && nextPoints[i].y < gray.rows) {

                goodPrev.push_back(prevPoints[i]);
                goodCurr.push_back(nextPoints[i]);
                goodIds.push_back(pointIds[i]);
                goodAges.push_back(pointAges[i] + 1);

                // Flow 벡터 저장
                frameData.flowVectors.push_back(nextPoints[i].x - prevPoints[i].x);
                frameData.flowVectors.push_back(nextPoints[i].y - prevPoints[i].y);

                // 매칭 인덱스 저장 (이전 인덱스, 현재 인덱스)
                frameData.matches.push_back(static_cast<int>(goodPrev.size() - 1));
                frameData.matches.push_back(static_cast<int>(goodCurr.size() - 1));
            }
        }
    }

    prevPoints = goodPrev;
    currPoints = goodCurr;
    pointIds = goodIds;
    pointAges = goodAges;

    frameData.matchCount = static_cast<int>(goodCurr.size());
}

// ============================================================================
// 포즈 추정 (Essential Matrix)
// ============================================================================

bool VisualOdometry::estimatePose() {
    if (prevPoints.size() < 8 || currPoints.size() < 8) {
        return false;
    }

    // Essential Matrix 계산
    cv::Mat mask;
    cv::Mat E = cv::findEssentialMat(
        prevPoints,
        currPoints,
        K,
        cv::RANSAC,
        config.ransacConfidence,
        config.ransacThreshold,
        mask
    );

    if (E.empty()) {
        return false;
    }

    // 포즈 복원
    cv::Mat R, t;
    int inliers = cv::recoverPose(E, prevPoints, currPoints, K, R, t, mask);

    if (inliers < config.minInliers) {
        return false;
    }

    // RANSAC 인라이어만 유지
    std::vector<cv::Point2f> inlierPrev;
    std::vector<cv::Point2f> inlierCurr;
    std::vector<int> inlierIds;
    std::vector<int> inlierAges;

    for (int i = 0; i < mask.rows; i++) {
        if (mask.at<uchar>(i)) {
            inlierPrev.push_back(prevPoints[i]);
            inlierCurr.push_back(currPoints[i]);
            if (i < (int)pointIds.size()) {
                inlierIds.push_back(pointIds[i]);
                inlierAges.push_back(pointAges[i]);
            }
        }
    }

    prevPoints = inlierPrev;
    currPoints = inlierCurr;
    pointIds = inlierIds;
    pointAges = inlierAges;

    // 포즈 누적
    // 단안 카메라는 스케일이 모호하므로 단위 스케일 사용
    t_total = t_total + scale * (R_total * t);
    R_total = R * R_total;

    // 포즈 데이터 업데이트
    rotationToQuaternion(R_total,
        frameData.pose.qx, frameData.pose.qy,
        frameData.pose.qz, frameData.pose.qw);

    frameData.pose.tx = static_cast<float>(t_total.at<double>(0));
    frameData.pose.ty = static_cast<float>(t_total.at<double>(1));
    frameData.pose.tz = static_cast<float>(t_total.at<double>(2));
    frameData.pose.confidence = static_cast<float>(inliers) / static_cast<float>(currPoints.size());
    frameData.pose.valid = true;

    return true;
}

// ============================================================================
// View Matrix 업데이트
// ============================================================================

void VisualOdometry::updateViewMatrix() {
    // OpenGL/WebGL View Matrix = [R | t]의 역행렬
    // View = [ R^T  | -R^T * t ]
    //        [ 0    |     1    ]

    cv::Mat R_t = R_total.t();
    cv::Mat t_view = -R_t * t_total;

    // Column-major order (WebGL)
    // Column 0
    frameData.viewMatrix[0] = static_cast<float>(R_t.at<double>(0, 0));
    frameData.viewMatrix[1] = static_cast<float>(R_t.at<double>(1, 0));
    frameData.viewMatrix[2] = static_cast<float>(R_t.at<double>(2, 0));
    frameData.viewMatrix[3] = 0.0f;

    // Column 1
    frameData.viewMatrix[4] = static_cast<float>(R_t.at<double>(0, 1));
    frameData.viewMatrix[5] = static_cast<float>(R_t.at<double>(1, 1));
    frameData.viewMatrix[6] = static_cast<float>(R_t.at<double>(2, 1));
    frameData.viewMatrix[7] = 0.0f;

    // Column 2
    frameData.viewMatrix[8] = static_cast<float>(R_t.at<double>(0, 2));
    frameData.viewMatrix[9] = static_cast<float>(R_t.at<double>(1, 2));
    frameData.viewMatrix[10] = static_cast<float>(R_t.at<double>(2, 2));
    frameData.viewMatrix[11] = 0.0f;

    // Column 3 (Translation)
    frameData.viewMatrix[12] = static_cast<float>(t_view.at<double>(0));
    frameData.viewMatrix[13] = static_cast<float>(t_view.at<double>(1));
    frameData.viewMatrix[14] = static_cast<float>(t_view.at<double>(2));
    frameData.viewMatrix[15] = 1.0f;
}

// ============================================================================
// 회전 행렬 → 쿼터니언 변환
// ============================================================================

void VisualOdometry::rotationToQuaternion(const cv::Mat& R,
    float& qx, float& qy, float& qz, float& qw) {

    double trace = R.at<double>(0, 0) + R.at<double>(1, 1) + R.at<double>(2, 2);

    if (trace > 0) {
        double s = 0.5 / std::sqrt(trace + 1.0);
        qw = static_cast<float>(0.25 / s);
        qx = static_cast<float>((R.at<double>(2, 1) - R.at<double>(1, 2)) * s);
        qy = static_cast<float>((R.at<double>(0, 2) - R.at<double>(2, 0)) * s);
        qz = static_cast<float>((R.at<double>(1, 0) - R.at<double>(0, 1)) * s);
    } else {
        if (R.at<double>(0, 0) > R.at<double>(1, 1) && R.at<double>(0, 0) > R.at<double>(2, 2)) {
            double s = 2.0 * std::sqrt(1.0 + R.at<double>(0, 0) - R.at<double>(1, 1) - R.at<double>(2, 2));
            qw = static_cast<float>((R.at<double>(2, 1) - R.at<double>(1, 2)) / s);
            qx = static_cast<float>(0.25 * s);
            qy = static_cast<float>((R.at<double>(0, 1) + R.at<double>(1, 0)) / s);
            qz = static_cast<float>((R.at<double>(0, 2) + R.at<double>(2, 0)) / s);
        } else if (R.at<double>(1, 1) > R.at<double>(2, 2)) {
            double s = 2.0 * std::sqrt(1.0 + R.at<double>(1, 1) - R.at<double>(0, 0) - R.at<double>(2, 2));
            qw = static_cast<float>((R.at<double>(0, 2) - R.at<double>(2, 0)) / s);
            qx = static_cast<float>((R.at<double>(0, 1) + R.at<double>(1, 0)) / s);
            qy = static_cast<float>(0.25 * s);
            qz = static_cast<float>((R.at<double>(1, 2) + R.at<double>(2, 1)) / s);
        } else {
            double s = 2.0 * std::sqrt(1.0 + R.at<double>(2, 2) - R.at<double>(0, 0) - R.at<double>(1, 1));
            qw = static_cast<float>((R.at<double>(1, 0) - R.at<double>(0, 1)) / s);
            qx = static_cast<float>((R.at<double>(0, 2) + R.at<double>(2, 0)) / s);
            qy = static_cast<float>((R.at<double>(1, 2) + R.at<double>(2, 1)) / s);
            qz = static_cast<float>(0.25 * s);
        }
    }

    // 정규화
    float len = std::sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
    if (len > 0.0001f) {
        qx /= len;
        qy /= len;
        qz /= len;
        qw /= len;
    }
}

// ============================================================================
// FrameData 업데이트 (JavaScript 전송용)
// ============================================================================

void VisualOdometry::updateFrameData() {
    // 특징점 좌표 (flat array)
    frameData.featurePositions.clear();
    frameData.featurePositions.reserve(currPoints.size() * 2);

    // 특징점 메타데이터 (flat array: [size, response, id, age, ...])
    frameData.featureMeta.clear();
    frameData.featureMeta.reserve(currPoints.size() * 4);

    for (size_t i = 0; i < currPoints.size(); i++) {
        // 좌표
        frameData.featurePositions.push_back(currPoints[i].x);
        frameData.featurePositions.push_back(currPoints[i].y);

        // 메타데이터
        frameData.featureMeta.push_back(7.0f);  // FAST는 size 정보가 없으므로 기본값
        frameData.featureMeta.push_back(1.0f);  // response 기본값
        frameData.featureMeta.push_back(static_cast<float>(i < pointIds.size() ? pointIds[i] : -1));
        frameData.featureMeta.push_back(static_cast<float>(i < pointAges.size() ? pointAges[i] : 0));
    }

    frameData.featureCount = static_cast<int>(currPoints.size());
}
