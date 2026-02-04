#include "plane_detector.h"
#include <iostream>
#include <random>
#include <algorithm>

PlaneDetector::PlaneDetector() {
    nextPlaneId = 0;
    std::cout << "[PlaneDetector] 초기화" << std::endl;
}

PlaneDetector::~PlaneDetector() {
    planes.clear();
}

bool PlaneDetector::detectPlanes(const cv::Mat& frame,
                                  const std::vector<cv::Point3f>& mapPoints,
                                  const cv::Mat& cameraPose) {
    if (mapPoints.size() < MIN_PLANE_POINTS) {
        return false;
    }

    // 맵 포인트들을 복사
    std::vector<cv::Point3f> remainingPoints = mapPoints;

    // 여러 평면 찾기 (최대 3개)
    for (int planeCount = 0; planeCount < 3 && remainingPoints.size() >= MIN_PLANE_POINTS; planeCount++) {
        cv::Point3f normal;
        float d;
        std::vector<int> inliers;

        if (!fitPlaneRANSAC(remainingPoints, normal, d, inliers)) {
            break;
        }

        if (inliers.size() < MIN_PLANE_POINTS) {
            break;
        }

        // 인라이어 포인트 추출
        std::vector<cv::Point3f> inlierPoints;
        for (int idx : inliers) {
            inlierPoints.push_back(remainingPoints[idx]);
        }

        // 평면 생성
        DetectedPlane plane;
        plane.id = nextPlaneId++;
        plane.normal = normal;
        plane.isHorizontal = isPlaneHorizontal(normal);
        plane.confidence = static_cast<float>(inliers.size()) / mapPoints.size();

        computePlaneBounds(inlierPoints, normal, plane);

        // 기존 평면과 병합 시도
        int mergeIdx = findMergeablePlane(plane);
        if (mergeIdx >= 0) {
            mergePlanes(mergeIdx, plane);
            nextPlaneId--; // ID 롤백
        } else {
            planes.push_back(plane);
            std::cout << "[PlaneDetector] 평면 감지 #" << plane.id
                      << (plane.isHorizontal ? " (수평)" : " (수직)")
                      << " 포인트: " << inliers.size()
                      << " 크기: " << plane.width << "x" << plane.height << "m"
                      << std::endl;
        }

        // 사용된 포인트 제거 (역순으로)
        std::sort(inliers.rbegin(), inliers.rend());
        for (int idx : inliers) {
            remainingPoints.erase(remainingPoints.begin() + idx);
        }
    }

    return !planes.empty();
}

bool PlaneDetector::fitPlaneRANSAC(const std::vector<cv::Point3f>& points,
                                    cv::Point3f& planeNormal, float& planeD,
                                    std::vector<int>& inliers) {
    if (points.size() < 3) return false;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, points.size() - 1);

    int bestInlierCount = 0;
    cv::Point3f bestNormal;
    float bestD = 0;

    for (int iter = 0; iter < RANSAC_ITERATIONS; iter++) {
        // 3개 랜덤 포인트 선택
        int i1 = dis(gen);
        int i2 = dis(gen);
        int i3 = dis(gen);

        if (i1 == i2 || i2 == i3 || i1 == i3) continue;

        cv::Point3f p1 = points[i1];
        cv::Point3f p2 = points[i2];
        cv::Point3f p3 = points[i3];

        // 평면 법선 계산 (외적)
        cv::Point3f v1 = p2 - p1;
        cv::Point3f v2 = p3 - p1;
        cv::Point3f normal = v1.cross(v2);

        float norm = cv::norm(normal);
        if (norm < 1e-6) continue;
        normal /= norm;

        // 평면 방정식: ax + by + cz + d = 0
        float d = -normal.dot(p1);

        // 인라이어 카운트
        std::vector<int> currentInliers;
        for (size_t i = 0; i < points.size(); i++) {
            float dist = std::abs(normal.dot(points[i]) + d);
            if (dist < RANSAC_THRESHOLD) {
                currentInliers.push_back(i);
            }
        }

        if (currentInliers.size() > bestInlierCount) {
            bestInlierCount = currentInliers.size();
            bestNormal = normal;
            bestD = d;
            inliers = currentInliers;
        }
    }

    if (bestInlierCount < MIN_PLANE_POINTS) {
        return false;
    }

    // 법선 방향 정규화 (위쪽을 향하도록)
    if (bestNormal.y < 0) {
        bestNormal = -bestNormal;
        bestD = -bestD;
    }

    planeNormal = bestNormal;
    planeD = bestD;

    return true;
}

bool PlaneDetector::isPlaneHorizontal(const cv::Point3f& normal) {
    // Y축(위쪽)과의 각도 확인
    cv::Point3f up(0, 1, 0);
    float dot = std::abs(normal.dot(up));
    return dot > HORIZONTAL_THRESHOLD;
}

void PlaneDetector::computePlaneBounds(const std::vector<cv::Point3f>& inlierPoints,
                                        const cv::Point3f& normal,
                                        DetectedPlane& plane) {
    if (inlierPoints.empty()) return;

    // 중심 계산
    cv::Point3f center(0, 0, 0);
    for (const auto& p : inlierPoints) {
        center += p;
    }
    center /= static_cast<float>(inlierPoints.size());
    plane.center = center;

    // 평면 좌표계 생성
    cv::Point3f up(0, 1, 0);
    cv::Point3f right = up.cross(normal);
    if (cv::norm(right) < 0.1f) {
        right = cv::Point3f(1, 0, 0);
    }
    right /= cv::norm(right);
    cv::Point3f forward = normal.cross(right);
    forward /= cv::norm(forward);

    // 2D 투영하여 경계 계산
    float minX = FLT_MAX, maxX = -FLT_MAX;
    float minZ = FLT_MAX, maxZ = -FLT_MAX;

    for (const auto& p : inlierPoints) {
        cv::Point3f local = p - center;
        float x = local.dot(right);
        float z = local.dot(forward);

        minX = std::min(minX, x);
        maxX = std::max(maxX, x);
        minZ = std::min(minZ, z);
        maxZ = std::max(maxZ, z);
    }

    plane.width = maxX - minX;
    plane.height = maxZ - minZ;

    // 4개 코너 계산
    plane.corners.clear();
    plane.corners.push_back(center + right * minX + forward * minZ);
    plane.corners.push_back(center + right * maxX + forward * minZ);
    plane.corners.push_back(center + right * maxX + forward * maxZ);
    plane.corners.push_back(center + right * minX + forward * maxZ);
}

int PlaneDetector::findMergeablePlane(const DetectedPlane& newPlane) {
    for (size_t i = 0; i < planes.size(); i++) {
        // 같은 타입(수평/수직)인지 확인
        if (planes[i].isHorizontal != newPlane.isHorizontal) continue;

        // 법선 방향이 비슷한지 확인
        float normalDot = planes[i].normal.dot(newPlane.normal);
        if (std::abs(normalDot) < 0.95f) continue;

        // 중심 거리 확인
        float dist = cv::norm(planes[i].center - newPlane.center);
        if (dist < MERGE_DISTANCE) {
            return i;
        }
    }
    return -1;
}

void PlaneDetector::mergePlanes(int existingIdx, const DetectedPlane& newPlane) {
    DetectedPlane& existing = planes[existingIdx];

    // 중심 평균
    existing.center = (existing.center + newPlane.center) * 0.5f;

    // 크기 확장
    existing.width = std::max(existing.width, newPlane.width);
    existing.height = std::max(existing.height, newPlane.height);

    // 신뢰도 업데이트
    existing.confidence = std::min(1.0f, existing.confidence + newPlane.confidence * 0.5f);

    std::cout << "[PlaneDetector] 평면 병합 #" << existing.id << std::endl;
}

std::vector<DetectedPlane> PlaneDetector::getDetectedPlanes() const {
    return planes;
}

bool PlaneDetector::hitTest(float screenX, float screenY,
                            const cv::Mat& cameraMatrix,
                            const cv::Mat& cameraPose,
                            cv::Point3f& hitPoint, int& planeId) {
    if (planes.empty()) return false;

    // 카메라 내부 파라미터
    double fx = cameraMatrix.at<double>(0, 0);
    double fy = cameraMatrix.at<double>(1, 1);
    double cx = cameraMatrix.at<double>(0, 2);
    double cy = cameraMatrix.at<double>(1, 2);

    // 스크린 좌표 → 정규화 좌표
    double nx = (screenX - cx) / fx;
    double ny = (screenY - cy) / fy;

    // 카메라 위치 및 방향
    cv::Mat R = cameraPose(cv::Rect(0, 0, 3, 3));
    cv::Mat t = cameraPose(cv::Rect(3, 0, 1, 3));

    cv::Point3f cameraPos(t.at<double>(0), t.at<double>(1), t.at<double>(2));

    // 레이 방향 (카메라 좌표계)
    cv::Mat rayLocal = (cv::Mat_<double>(3, 1) << nx, ny, 1.0);
    cv::Mat rayWorld = R * rayLocal;
    cv::Point3f rayDir(rayWorld.at<double>(0), rayWorld.at<double>(1), rayWorld.at<double>(2));
    rayDir /= cv::norm(rayDir);

    // 가장 가까운 평면과 교차점 찾기
    float minDist = FLT_MAX;
    bool found = false;

    for (const auto& plane : planes) {
        // 레이-평면 교차
        float denom = rayDir.dot(plane.normal);
        if (std::abs(denom) < 1e-6) continue;

        float t_param = (plane.center - cameraPos).dot(plane.normal) / denom;
        if (t_param < 0) continue; // 뒤쪽은 무시

        cv::Point3f intersection = cameraPos + rayDir * t_param;

        // 평면 경계 내인지 확인 (간단한 거리 체크)
        float distToCenter = cv::norm(intersection - plane.center);
        float maxDist = std::max(plane.width, plane.height) * 0.6f;

        if (distToCenter < maxDist && t_param < minDist) {
            minDist = t_param;
            hitPoint = intersection;
            planeId = plane.id;
            found = true;
        }
    }

    return found;
}
