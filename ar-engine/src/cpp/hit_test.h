#ifndef HIT_TEST_H
#define HIT_TEST_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>

/**
 * 3D 평면 구조체
 * 평면 방정식: ax + by + cz + d = 0
 * 또는 normal·P + d = 0
 */
struct Plane3D {
    float nx, ny, nz;    // 법선 벡터 (정규화됨)
    float d;             // 평면 상수 (원점에서 평면까지 부호 있는 거리)

    // 점이 평면에서 얼마나 떨어져 있는지 (부호 있는 거리)
    float signedDistance(float x, float y, float z) const {
        return nx * x + ny * y + nz * z + d;
    }

    // 점이 평면 위에 있는지 (허용 오차 내)
    bool isOnPlane(float x, float y, float z, float threshold = 0.02f) const {
        return std::abs(signedDistance(x, y, z)) < threshold;
    }
};

/**
 * 레이 구조체 (광선)
 */
struct Ray3D {
    float ox, oy, oz;    // 원점 (Origin)
    float dx, dy, dz;    // 방향 (Direction, 정규화됨)

    // t 파라미터에서의 점 계산
    void pointAt(float t, float& x, float& y, float& z) const {
        x = ox + dx * t;
        y = oy + dy * t;
        z = oz + dz * t;
    }
};

/**
 * Hit Test 결과
 */
struct HitTestResult {
    bool hit;               // 충돌 여부
    float x, y, z;          // 충돌 지점 (월드 좌표)
    float distance;         // 레이 원점에서 충돌 지점까지 거리
    int planeId;            // 충돌한 평면 ID (-1 = 가상 바닥)
    float confidence;       // 신뢰도 (0~1)

    HitTestResult() : hit(false), x(0), y(0), z(0), distance(0), planeId(-1), confidence(0) {}
};

/**
 * HitTester 클래스
 * 특징점 기반 평면 추정 + 레이캐스팅
 */
class HitTester {
public:
    HitTester();
    ~HitTester();

    // ========================================
    // 평면 추정 (RANSAC)
    // ========================================

    /**
     * 3D 포인트 클라우드에서 수평면 추정
     * @param points 3D 특징점 배열
     * @param pointCount 특징점 개수
     * @return 추정 성공 여부
     */
    bool estimateGroundPlane(const float* points, int pointCount);

    /**
     * 수동으로 바닥 평면 설정 (y=0 평면)
     */
    void setDefaultGroundPlane();

    /**
     * 현재 추정된 바닥 평면 반환
     */
    Plane3D getGroundPlane() const { return groundPlane; }
    bool hasGroundPlane() const { return hasValidPlane; }

    // ========================================
    // Hit Test (레이캐스팅)
    // ========================================

    /**
     * 스크린 좌표에서 레이 생성
     * @param screenX, screenY 스크린 좌표 (픽셀)
     * @param screenWidth, screenHeight 스크린 크기
     * @param viewMatrix 4x4 뷰 행렬 (column-major)
     * @param projMatrix 4x4 투영 행렬 (column-major)
     */
    Ray3D screenToRay(float screenX, float screenY,
                      int screenWidth, int screenHeight,
                      const float* viewMatrix,
                      const float* projMatrix);

    /**
     * 레이와 평면의 교차점 계산
     * @param ray 레이
     * @param plane 평면
     * @param result 결과 (출력)
     * @return 교차 여부
     */
    bool rayPlaneIntersect(const Ray3D& ray, const Plane3D& plane, HitTestResult& result);

    /**
     * Hit Test 실행 (스크린 좌표 → 바닥 월드 좌표)
     * @param screenX, screenY 터치 좌표 (픽셀)
     * @param screenWidth, screenHeight 스크린 크기
     * @param viewMatrix 4x4 뷰 행렬
     * @param projMatrix 4x4 투영 행렬
     * @return HitTestResult
     */
    HitTestResult hitTest(float screenX, float screenY,
                          int screenWidth, int screenHeight,
                          const float* viewMatrix,
                          const float* projMatrix);

    // ========================================
    // 유틸리티
    // ========================================

    /**
     * 카메라 FOV로 투영 행렬 생성
     */
    static void createProjectionMatrix(float fovY, float aspect, float near, float far, float* out);

    /**
     * 4x4 행렬 역행렬 계산
     */
    static bool invertMatrix4x4(const float* m, float* out);

private:
    /**
     * RANSAC 평면 피팅
     */
    bool fitPlaneRANSAC(const std::vector<cv::Point3f>& points, Plane3D& outPlane,
                        std::vector<int>& inliers);

    /**
     * 3점에서 평면 계산
     */
    bool computePlaneFromPoints(const cv::Point3f& p1, const cv::Point3f& p2,
                                const cv::Point3f& p3, Plane3D& outPlane);

    /**
     * 평면이 수평인지 확인 (y축과 법선이 평행)
     */
    bool isHorizontalPlane(const Plane3D& plane) const;

private:
    Plane3D groundPlane;
    bool hasValidPlane;

    // RANSAC 설정
    int ransacIterations;
    float ransacThreshold;
    int minInliers;
    float horizontalThreshold;  // cos(각도)
};

#endif // HIT_TEST_H
