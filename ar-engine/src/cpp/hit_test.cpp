#include "hit_test.h"
#include <random>
#include <algorithm>
#include <cstring>

// ============================================================================
// 생성자 / 소멸자
// ============================================================================

HitTester::HitTester()
    : hasValidPlane(false)
    , ransacIterations(100)
    , ransacThreshold(0.03f)     // 3cm
    , minInliers(20)
    , horizontalThreshold(0.85f) // cos(~32도)
{
    setDefaultGroundPlane();
}

HitTester::~HitTester() {}

// ============================================================================
// 평면 추정
// ============================================================================

void HitTester::setDefaultGroundPlane() {
    // 기본 바닥: y = 0 평면 (법선이 위쪽을 향함)
    groundPlane.nx = 0.0f;
    groundPlane.ny = 1.0f;
    groundPlane.nz = 0.0f;
    groundPlane.d = 0.0f;
    hasValidPlane = true;
}

bool HitTester::estimateGroundPlane(const float* points, int pointCount) {
    if (!points || pointCount < minInliers) {
        return false;
    }

    // float 배열 → Point3f 벡터
    std::vector<cv::Point3f> pts;
    pts.reserve(pointCount);

    for (int i = 0; i < pointCount; i++) {
        pts.emplace_back(
            points[i * 3 + 0],
            points[i * 3 + 1],
            points[i * 3 + 2]
        );
    }

    std::vector<int> inliers;
    Plane3D candidatePlane;

    if (!fitPlaneRANSAC(pts, candidatePlane, inliers)) {
        return false;
    }

    // 수평면인지 확인
    if (!isHorizontalPlane(candidatePlane)) {
        return false;
    }

    // 법선이 위쪽을 향하도록 보장 (y > 0)
    if (candidatePlane.ny < 0) {
        candidatePlane.nx = -candidatePlane.nx;
        candidatePlane.ny = -candidatePlane.ny;
        candidatePlane.nz = -candidatePlane.nz;
        candidatePlane.d = -candidatePlane.d;
    }

    groundPlane = candidatePlane;
    hasValidPlane = true;

    return true;
}

bool HitTester::fitPlaneRANSAC(const std::vector<cv::Point3f>& points,
                               Plane3D& outPlane, std::vector<int>& inliers) {
    if (points.size() < 3) return false;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, static_cast<int>(points.size()) - 1);

    int bestInlierCount = 0;
    Plane3D bestPlane;

    for (int iter = 0; iter < ransacIterations; iter++) {
        // 랜덤하게 3점 선택
        int i1 = dis(gen);
        int i2 = dis(gen);
        int i3 = dis(gen);

        // 중복 방지
        if (i1 == i2 || i2 == i3 || i1 == i3) continue;

        // 3점으로 평면 계산
        Plane3D plane;
        if (!computePlaneFromPoints(points[i1], points[i2], points[i3], plane)) {
            continue;
        }

        // 인라이어 카운트
        int inlierCount = 0;
        for (size_t i = 0; i < points.size(); i++) {
            float dist = std::abs(plane.signedDistance(points[i].x, points[i].y, points[i].z));
            if (dist < ransacThreshold) {
                inlierCount++;
            }
        }

        if (inlierCount > bestInlierCount) {
            bestInlierCount = inlierCount;
            bestPlane = plane;
        }
    }

    if (bestInlierCount < minInliers) {
        return false;
    }

    // 인라이어 수집
    inliers.clear();
    for (size_t i = 0; i < points.size(); i++) {
        float dist = std::abs(bestPlane.signedDistance(points[i].x, points[i].y, points[i].z));
        if (dist < ransacThreshold) {
            inliers.push_back(static_cast<int>(i));
        }
    }

    outPlane = bestPlane;
    return true;
}

bool HitTester::computePlaneFromPoints(const cv::Point3f& p1, const cv::Point3f& p2,
                                       const cv::Point3f& p3, Plane3D& outPlane) {
    // 두 벡터 계산
    cv::Point3f v1 = p2 - p1;
    cv::Point3f v2 = p3 - p1;

    // 외적으로 법선 계산
    cv::Point3f normal;
    normal.x = v1.y * v2.z - v1.z * v2.y;
    normal.y = v1.z * v2.x - v1.x * v2.z;
    normal.z = v1.x * v2.y - v1.y * v2.x;

    // 정규화
    float len = std::sqrt(normal.x * normal.x + normal.y * normal.y + normal.z * normal.z);
    if (len < 1e-6f) return false;

    outPlane.nx = normal.x / len;
    outPlane.ny = normal.y / len;
    outPlane.nz = normal.z / len;

    // d = -(n · p1)
    outPlane.d = -(outPlane.nx * p1.x + outPlane.ny * p1.y + outPlane.nz * p1.z);

    return true;
}

bool HitTester::isHorizontalPlane(const Plane3D& plane) const {
    // y축 (0, 1, 0)과 법선의 내적 = ny
    // 수평면이면 법선이 위/아래를 향함 → |ny| ≈ 1
    return std::abs(plane.ny) > horizontalThreshold;
}

// ============================================================================
// Hit Test (레이캐스팅)
// ============================================================================

Ray3D HitTester::screenToRay(float screenX, float screenY,
                              int screenWidth, int screenHeight,
                              const float* viewMatrix,
                              const float* projMatrix) {
    Ray3D ray;

    // 1. 스크린 좌표 → NDC (Normalized Device Coordinates)
    // NDC 범위: x,y ∈ [-1, 1]
    float ndcX = (2.0f * screenX / screenWidth) - 1.0f;
    float ndcY = 1.0f - (2.0f * screenY / screenHeight);  // Y축 반전

    // 2. NDC → 클립 공간 (near plane과 far plane)
    // near: z = -1, far: z = 1 (OpenGL NDC)
    float nearPoint[4] = { ndcX, ndcY, -1.0f, 1.0f };
    float farPoint[4] = { ndcX, ndcY, 1.0f, 1.0f };

    // 3. 투영 행렬 역행렬
    float invProj[16];
    if (!invertMatrix4x4(projMatrix, invProj)) {
        // 실패 시 기본 레이 반환
        ray.ox = 0; ray.oy = 0; ray.oz = 0;
        ray.dx = 0; ray.dy = 0; ray.dz = -1;
        return ray;
    }

    // 4. 뷰 행렬 역행렬 (= 카메라 월드 변환)
    float invView[16];
    if (!invertMatrix4x4(viewMatrix, invView)) {
        ray.ox = 0; ray.oy = 0; ray.oz = 0;
        ray.dx = 0; ray.dy = 0; ray.dz = -1;
        return ray;
    }

    // 5. 클립 → 뷰 공간 변환 (invProj 적용)
    auto multiplyMat4Vec4 = [](const float* m, const float* v, float* out) {
        for (int i = 0; i < 4; i++) {
            out[i] = m[i] * v[0] + m[i + 4] * v[1] + m[i + 8] * v[2] + m[i + 12] * v[3];
        }
    };

    float nearView[4], farView[4];
    multiplyMat4Vec4(invProj, nearPoint, nearView);
    multiplyMat4Vec4(invProj, farPoint, farView);

    // Perspective divide
    if (std::abs(nearView[3]) > 1e-6f) {
        nearView[0] /= nearView[3];
        nearView[1] /= nearView[3];
        nearView[2] /= nearView[3];
    }
    if (std::abs(farView[3]) > 1e-6f) {
        farView[0] /= farView[3];
        farView[1] /= farView[3];
        farView[2] /= farView[3];
    }

    // 6. 뷰 → 월드 공간 변환 (invView 적용)
    float nearWorld[4] = { nearView[0], nearView[1], nearView[2], 1.0f };
    float farWorld[4] = { farView[0], farView[1], farView[2], 1.0f };
    float nearW[4], farW[4];
    multiplyMat4Vec4(invView, nearWorld, nearW);
    multiplyMat4Vec4(invView, farWorld, farW);

    // 7. 레이 구성
    ray.ox = nearW[0];
    ray.oy = nearW[1];
    ray.oz = nearW[2];

    float dx = farW[0] - nearW[0];
    float dy = farW[1] - nearW[1];
    float dz = farW[2] - nearW[2];
    float len = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (len > 1e-6f) {
        ray.dx = dx / len;
        ray.dy = dy / len;
        ray.dz = dz / len;
    } else {
        ray.dx = 0;
        ray.dy = 0;
        ray.dz = -1;
    }

    return ray;
}

bool HitTester::rayPlaneIntersect(const Ray3D& ray, const Plane3D& plane, HitTestResult& result) {
    // 레이-평면 교차 공식:
    // 레이: P = O + t * D
    // 평면: N · P + d = 0
    //
    // N · (O + t * D) + d = 0
    // t = -(N · O + d) / (N · D)

    float nDotD = plane.nx * ray.dx + plane.ny * ray.dy + plane.nz * ray.dz;

    // 레이가 평면과 평행한 경우
    if (std::abs(nDotD) < 1e-6f) {
        result.hit = false;
        return false;
    }

    float nDotO = plane.nx * ray.ox + plane.ny * ray.oy + plane.nz * ray.oz;
    float t = -(nDotO + plane.d) / nDotD;

    // t < 0이면 레이 뒤쪽에 교차점이 있음 (카메라 뒤)
    if (t < 0) {
        result.hit = false;
        return false;
    }

    // 교차점 계산
    ray.pointAt(t, result.x, result.y, result.z);
    result.distance = t;
    result.hit = true;
    result.planeId = -1;  // 기본 바닥 평면
    result.confidence = 1.0f;

    return true;
}

HitTestResult HitTester::hitTest(float screenX, float screenY,
                                  int screenWidth, int screenHeight,
                                  const float* viewMatrix,
                                  const float* projMatrix) {
    HitTestResult result;

    if (!hasValidPlane) {
        result.hit = false;
        return result;
    }

    // 스크린 좌표 → 레이
    Ray3D ray = screenToRay(screenX, screenY, screenWidth, screenHeight, viewMatrix, projMatrix);

    // 레이-평면 교차
    rayPlaneIntersect(ray, groundPlane, result);

    return result;
}

// ============================================================================
// 유틸리티
// ============================================================================

void HitTester::createProjectionMatrix(float fovY, float aspect, float near, float far, float* out) {
    float tanHalfFov = std::tan(fovY / 2.0f);

    std::memset(out, 0, 16 * sizeof(float));

    out[0] = 1.0f / (aspect * tanHalfFov);
    out[5] = 1.0f / tanHalfFov;
    out[10] = -(far + near) / (far - near);
    out[11] = -1.0f;
    out[14] = -(2.0f * far * near) / (far - near);
}

bool HitTester::invertMatrix4x4(const float* m, float* out) {
    float inv[16];

    inv[0] = m[5] * m[10] * m[15] - m[5] * m[11] * m[14] - m[9] * m[6] * m[15] +
             m[9] * m[7] * m[14] + m[13] * m[6] * m[11] - m[13] * m[7] * m[10];

    inv[4] = -m[4] * m[10] * m[15] + m[4] * m[11] * m[14] + m[8] * m[6] * m[15] -
              m[8] * m[7] * m[14] - m[12] * m[6] * m[11] + m[12] * m[7] * m[10];

    inv[8] = m[4] * m[9] * m[15] - m[4] * m[11] * m[13] - m[8] * m[5] * m[15] +
             m[8] * m[7] * m[13] + m[12] * m[5] * m[11] - m[12] * m[7] * m[9];

    inv[12] = -m[4] * m[9] * m[14] + m[4] * m[10] * m[13] + m[8] * m[5] * m[14] -
               m[8] * m[6] * m[13] - m[12] * m[5] * m[10] + m[12] * m[6] * m[9];

    inv[1] = -m[1] * m[10] * m[15] + m[1] * m[11] * m[14] + m[9] * m[2] * m[15] -
              m[9] * m[3] * m[14] - m[13] * m[2] * m[11] + m[13] * m[3] * m[10];

    inv[5] = m[0] * m[10] * m[15] - m[0] * m[11] * m[14] - m[8] * m[2] * m[15] +
             m[8] * m[3] * m[14] + m[12] * m[2] * m[11] - m[12] * m[3] * m[10];

    inv[9] = -m[0] * m[9] * m[15] + m[0] * m[11] * m[13] + m[8] * m[1] * m[15] -
              m[8] * m[3] * m[13] - m[12] * m[1] * m[11] + m[12] * m[3] * m[9];

    inv[13] = m[0] * m[9] * m[14] - m[0] * m[10] * m[13] - m[8] * m[1] * m[14] +
              m[8] * m[2] * m[13] + m[12] * m[1] * m[10] - m[12] * m[2] * m[9];

    inv[2] = m[1] * m[6] * m[15] - m[1] * m[7] * m[14] - m[5] * m[2] * m[15] +
             m[5] * m[3] * m[14] + m[13] * m[2] * m[7] - m[13] * m[3] * m[6];

    inv[6] = -m[0] * m[6] * m[15] + m[0] * m[7] * m[14] + m[4] * m[2] * m[15] -
              m[4] * m[3] * m[14] - m[12] * m[2] * m[7] + m[12] * m[3] * m[6];

    inv[10] = m[0] * m[5] * m[15] - m[0] * m[7] * m[13] - m[4] * m[1] * m[15] +
              m[4] * m[3] * m[13] + m[12] * m[1] * m[7] - m[12] * m[3] * m[5];

    inv[14] = -m[0] * m[5] * m[14] + m[0] * m[6] * m[13] + m[4] * m[1] * m[14] -
               m[4] * m[2] * m[13] - m[12] * m[1] * m[6] + m[12] * m[2] * m[5];

    inv[3] = -m[1] * m[6] * m[11] + m[1] * m[7] * m[10] + m[5] * m[2] * m[11] -
              m[5] * m[3] * m[10] - m[9] * m[2] * m[7] + m[9] * m[3] * m[6];

    inv[7] = m[0] * m[6] * m[11] - m[0] * m[7] * m[10] - m[4] * m[2] * m[11] +
             m[4] * m[3] * m[10] + m[8] * m[2] * m[7] - m[8] * m[3] * m[6];

    inv[11] = -m[0] * m[5] * m[11] + m[0] * m[7] * m[9] + m[4] * m[1] * m[11] -
               m[4] * m[3] * m[9] - m[8] * m[1] * m[7] + m[8] * m[3] * m[5];

    inv[15] = m[0] * m[5] * m[10] - m[0] * m[6] * m[9] - m[4] * m[1] * m[10] +
              m[4] * m[2] * m[9] + m[8] * m[1] * m[6] - m[8] * m[2] * m[5];

    float det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];

    if (std::abs(det) < 1e-10f) {
        return false;
    }

    det = 1.0f / det;

    for (int i = 0; i < 16; i++) {
        out[i] = inv[i] * det;
    }

    return true;
}
