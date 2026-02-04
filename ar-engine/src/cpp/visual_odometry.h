#ifndef VISUAL_ODOMETRY_H
#define VISUAL_ODOMETRY_H

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <vector>
#include <deque>
#include <memory>

/**
 * FeaturePoint - 특징점 데이터 구조체
 * JavaScript로 효율적인 전송을 위해 설계
 */
struct FeaturePoint {
    float x;            // 이미지 x 좌표
    float y;            // 이미지 y 좌표
    float size;         // 특징점 크기
    float response;     // 응답 강도 (품질)
    int id;             // 트래킹 ID (-1 = 새 특징점)
    int age;            // 몇 프레임 동안 추적되었는지

    FeaturePoint() : x(0), y(0), size(0), response(0), id(-1), age(0) {}
    FeaturePoint(float _x, float _y, float _size, float _response, int _id = -1)
        : x(_x), y(_y), size(_size), response(_response), id(_id), age(0) {}
};

/**
 * PoseData - 카메라 포즈 데이터
 */
struct PoseData {
    // 회전 (쿼터니언)
    float qx, qy, qz, qw;
    // 이동
    float tx, ty, tz;
    // 신뢰도 (0~1)
    float confidence;
    // 유효 여부
    bool valid;

    PoseData() : qx(0), qy(0), qz(0), qw(1), tx(0), ty(0), tz(0),
                 confidence(0), valid(false) {}
};

/**
 * FrameData - 프레임 처리 결과
 * JavaScript로 한 번에 전송하기 위한 구조체
 */
struct FrameData {
    // 특징점 (flat array: [x0,y0,x1,y1,...])
    std::vector<float> featurePositions;
    // 특징점 메타데이터 (flat array: [size0,response0,id0,age0,...])
    std::vector<float> featureMeta;
    // 특징점 개수
    int featureCount;

    // 매칭된 특징점 쌍 (flat array: [prevIdx0,currIdx0,prevIdx1,currIdx1,...])
    std::vector<int> matches;
    int matchCount;

    // Optical Flow 벡터 (flat array: [dx0,dy0,dx1,dy1,...])
    std::vector<float> flowVectors;

    // 포즈 추정 결과
    PoseData pose;

    // 4x4 View Matrix (column-major for WebGL)
    float viewMatrix[16];

    // 상태
    bool initialized;
    bool tracking;
    int frameNumber;
    float processingTimeMs;

    FrameData() : featureCount(0), matchCount(0), initialized(false),
                  tracking(false), frameNumber(0), processingTimeMs(0) {
        std::fill(viewMatrix, viewMatrix + 16, 0.0f);
        viewMatrix[0] = viewMatrix[5] = viewMatrix[10] = viewMatrix[15] = 1.0f;
    }
};

/**
 * VisualOdometry - SLAM 코어 클래스
 *
 * FAST 특징점 추출 + Optical Flow + 포즈 추정
 * Emscripten을 통해 JavaScript에서 호출 가능
 */
class VisualOdometry {
public:
    /**
     * 설정 옵션
     */
    struct Config {
        // FAST 검출기 설정
        int fastThreshold = 20;         // FAST 코너 임계값 (10~50)
        bool fastNonmaxSuppression = true;
        int maxFeatures = 500;          // 최대 특징점 수

        // Optical Flow 설정
        int lkWinSize = 21;             // Lucas-Kanade 윈도우 크기
        int lkMaxLevel = 3;             // 피라미드 레벨
        int lkMaxIter = 30;             // 최대 반복 횟수
        float lkEpsilon = 0.01f;        // 수렴 조건

        // 포즈 추정 설정
        float ransacThreshold = 1.0f;   // RANSAC 임계값 (픽셀)
        float ransacConfidence = 0.999f;
        int minInliers = 20;            // 최소 인라이어 수

        // 카메라 설정 (기본값: 720p 추정)
        float focalLength = 800.0f;
        float cx = 640.0f;              // 주점 x
        float cy = 360.0f;              // 주점 y
    };

    VisualOdometry();
    explicit VisualOdometry(const Config& config);
    ~VisualOdometry();

    /**
     * 설정 변경
     */
    void setConfig(const Config& config);
    Config getConfig() const { return config; }

    /**
     * 카메라 파라미터 설정
     * @param fx, fy: 초점 거리
     * @param cx, cy: 주점
     */
    void setCameraParams(float fx, float fy, float cx, float cy);

    /**
     * 메인 처리 함수 - 프레임 처리
     * @param gray: 그레이스케일 이미지 (CV_8UC1)
     * @return 처리 성공 여부
     */
    bool processFrame(const cv::Mat& gray);

    /**
     * RGBA 이미지 직접 처리 (JS에서 호출 용이)
     * @param width, height: 이미지 크기
     * @param data: RGBA 픽셀 데이터 (width * height * 4)
     */
    bool processFrameRGBA(int width, int height, const uint8_t* data);

    /**
     * 리셋
     */
    void reset();

    // ========================================
    // 결과 반환 (JavaScript 전송용)
    // ========================================

    /**
     * 전체 프레임 데이터 반환
     */
    const FrameData& getFrameData() const { return frameData; }

    /**
     * 특징점 좌표 배열 반환 (flat: [x0,y0,x1,y1,...])
     * JavaScript에서 Float32Array로 직접 사용 가능
     */
    const std::vector<float>& getFeaturePositions() const {
        return frameData.featurePositions;
    }

    /**
     * 특징점 메타데이터 배열
     */
    const std::vector<float>& getFeatureMeta() const {
        return frameData.featureMeta;
    }

    /**
     * 특징점 개수
     */
    int getFeatureCount() const { return frameData.featureCount; }

    /**
     * 매칭 인덱스 배열 반환
     */
    const std::vector<int>& getMatches() const { return frameData.matches; }
    int getMatchCount() const { return frameData.matchCount; }

    /**
     * Optical Flow 벡터 배열
     */
    const std::vector<float>& getFlowVectors() const {
        return frameData.flowVectors;
    }

    /**
     * 4x4 View Matrix (column-major)
     */
    const float* getViewMatrix() const { return frameData.viewMatrix; }

    /**
     * 포즈 데이터
     */
    const PoseData& getPose() const { return frameData.pose; }

    /**
     * 상태 조회
     */
    bool isInitialized() const { return frameData.initialized; }
    bool isTracking() const { return frameData.tracking; }
    int getFrameNumber() const { return frameData.frameNumber; }
    float getProcessingTime() const { return frameData.processingTimeMs; }

private:
    /**
     * FAST 특징점 추출
     */
    void extractFeatures(const cv::Mat& gray);

    /**
     * Optical Flow 계산
     */
    void trackFeatures(const cv::Mat& gray);

    /**
     * Essential Matrix로 포즈 추정
     */
    bool estimatePose();

    /**
     * View Matrix 업데이트
     */
    void updateViewMatrix();

    /**
     * 회전 행렬 → 쿼터니언 변환
     */
    void rotationToQuaternion(const cv::Mat& R, float& qx, float& qy, float& qz, float& qw);

    /**
     * FrameData 업데이트 (JavaScript 전송용)
     */
    void updateFrameData();

private:
    Config config;

    // 카메라 내부 파라미터
    cv::Mat K;              // 3x3 카메라 행렬
    cv::Mat distCoeffs;     // 왜곡 계수 (현재 미사용)

    // FAST 검출기
    cv::Ptr<cv::FastFeatureDetector> fastDetector;

    // 현재/이전 프레임
    cv::Mat prevGray;
    cv::Mat currGray;

    // 현재/이전 특징점
    std::vector<cv::Point2f> prevPoints;
    std::vector<cv::Point2f> currPoints;
    std::vector<int> pointIds;      // 각 특징점의 트래킹 ID
    std::vector<int> pointAges;     // 각 특징점이 추적된 프레임 수

    // 누적 포즈
    cv::Mat R_total;        // 3x3 총 회전
    cv::Mat t_total;        // 3x1 총 이동
    double scale;           // 스케일 (단안 카메라는 스케일 모호성 존재)

    // 출력 데이터
    FrameData frameData;

    // 내부 상태
    int frameCount;
    int nextPointId;
    bool initialized;

    // 타이밍
    double lastProcessTime;
};

#endif // VISUAL_ODOMETRY_H
