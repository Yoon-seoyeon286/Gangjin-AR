/**
 * vo_bindings.cpp
 *
 * VisualOdometry 클래스를 JavaScript에서 호출할 수 있도록
 * Emscripten 바인딩을 정의
 *
 * 특징점 데이터를 효율적으로 JS로 전송하기 위해:
 * 1. TypedArray 직접 접근 (Zero-copy 목표)
 * 2. Flat array 구조 사용 (JSON 파싱 오버헤드 제거)
 * 3. 한 번의 호출로 모든 데이터 전송
 */

#include <emscripten/bind.h>
#include <emscripten/val.h>
#include "visual_odometry.h"
#include <vector>

using namespace emscripten;

// ============================================================================
// 효율적인 배열 전송을 위한 헬퍼 함수
// ============================================================================

/**
 * std::vector<float> → JavaScript Float32Array
 * Emscripten의 typed_memory_view를 사용하여 복사 최소화
 */
val vectorToFloat32Array(const std::vector<float>& vec) {
    if (vec.empty()) {
        return val::array();
    }
    // typed_memory_view: C++ 메모리를 직접 참조하는 JS TypedArray 뷰 생성
    // 주의: C++ 측 메모리가 유효한 동안만 사용 가능
    return val(typed_memory_view(vec.size(), vec.data()));
}

/**
 * std::vector<int> → JavaScript Int32Array
 */
val vectorToInt32Array(const std::vector<int>& vec) {
    if (vec.empty()) {
        return val::array();
    }
    return val(typed_memory_view(vec.size(), vec.data()));
}

/**
 * float[16] → JavaScript Float32Array (View Matrix용)
 */
val arrayToFloat32Array(const float* arr, size_t size) {
    return val(typed_memory_view(size, arr));
}

// ============================================================================
// VisualOdometry 래퍼 클래스
// ============================================================================

class VisualOdometryWrapper {
public:
    VisualOdometryWrapper() : vo() {}

    /**
     * 설정 적용
     */
    void configure(int fastThreshold, int maxFeatures, float focalLength, float cx, float cy) {
        VisualOdometry::Config config;
        config.fastThreshold = fastThreshold;
        config.maxFeatures = maxFeatures;
        config.focalLength = focalLength;
        config.cx = cx;
        config.cy = cy;
        vo.setConfig(config);
    }

    /**
     * 카메라 파라미터 설정
     */
    void setCameraParams(float fx, float fy, float cx, float cy) {
        vo.setCameraParams(fx, fy, cx, cy);
    }

    /**
     * 프레임 처리 (RGBA 이미지 데이터)
     * JavaScript에서 ImageData.data를 직접 전달
     */
    bool processFrame(int width, int height, const val& imageData) {
        // JavaScript Uint8Array → C++ vector
        std::vector<uint8_t> buffer = vecFromJSArray<uint8_t>(imageData);

        if (buffer.size() != (size_t)(width * height * 4)) {
            return false;
        }

        return vo.processFrameRGBA(width, height, buffer.data());
    }

    /**
     * 리셋
     */
    void reset() {
        vo.reset();
    }

    // ========================================
    // 결과 반환 (효율적인 TypedArray 전송)
    // ========================================

    /**
     * 특징점 좌표 배열 [x0, y0, x1, y1, ...]
     * @returns Float32Array
     */
    val getFeaturePositions() {
        return vectorToFloat32Array(vo.getFeaturePositions());
    }

    /**
     * 특징점 메타데이터 [size0, response0, id0, age0, ...]
     * @returns Float32Array
     */
    val getFeatureMeta() {
        return vectorToFloat32Array(vo.getFeatureMeta());
    }

    /**
     * 특징점 개수
     */
    int getFeatureCount() {
        return vo.getFeatureCount();
    }

    /**
     * 매칭 인덱스 배열 [prevIdx0, currIdx0, ...]
     * @returns Int32Array
     */
    val getMatches() {
        return vectorToInt32Array(vo.getMatches());
    }

    /**
     * 매칭 개수
     */
    int getMatchCount() {
        return vo.getMatchCount();
    }

    /**
     * Optical Flow 벡터 [dx0, dy0, dx1, dy1, ...]
     * @returns Float32Array
     */
    val getFlowVectors() {
        return vectorToFloat32Array(vo.getFlowVectors());
    }

    /**
     * 4x4 View Matrix (column-major, WebGL용)
     * @returns Float32Array (16개 요소)
     */
    val getViewMatrix() {
        return arrayToFloat32Array(vo.getViewMatrix(), 16);
    }

    /**
     * 포즈 데이터 (Object로 반환)
     */
    val getPose() {
        const PoseData& pose = vo.getPose();

        val result = val::object();
        result.set("qx", pose.qx);
        result.set("qy", pose.qy);
        result.set("qz", pose.qz);
        result.set("qw", pose.qw);
        result.set("tx", pose.tx);
        result.set("ty", pose.ty);
        result.set("tz", pose.tz);
        result.set("confidence", pose.confidence);
        result.set("valid", pose.valid);

        return result;
    }

    /**
     * 전체 프레임 데이터 (한 번의 호출로 모든 데이터)
     * JavaScript에서 구조 분해 할당으로 사용
     */
    val getFrameData() {
        val result = val::object();

        // 특징점
        result.set("featurePositions", getFeaturePositions());
        result.set("featureMeta", getFeatureMeta());
        result.set("featureCount", getFeatureCount());

        // 매칭
        result.set("matches", getMatches());
        result.set("matchCount", getMatchCount());

        // Flow
        result.set("flowVectors", getFlowVectors());

        // 포즈
        result.set("pose", getPose());

        // View Matrix
        result.set("viewMatrix", getViewMatrix());

        // 상태
        result.set("initialized", vo.isInitialized());
        result.set("tracking", vo.isTracking());
        result.set("frameNumber", vo.getFrameNumber());
        result.set("processingTimeMs", vo.getProcessingTime());

        return result;
    }

    /**
     * 상태 조회
     */
    bool isInitialized() { return vo.isInitialized(); }
    bool isTracking() { return vo.isTracking(); }
    int getFrameNumber() { return vo.getFrameNumber(); }
    float getProcessingTime() { return vo.getProcessingTime(); }

private:
    VisualOdometry vo;
};

// ============================================================================
// 유틸리티 함수
// ============================================================================

/**
 * 특징점을 화면에 그리기 위한 데이터 생성
 * Canvas 2D Context에서 직접 사용 가능한 형태
 */
val createFeatureDrawData(const val& positions, int count, float pointSize) {
    val result = val::array();

    std::vector<float> posVec = vecFromJSArray<float>(positions);

    for (int i = 0; i < count && i * 2 + 1 < (int)posVec.size(); i++) {
        val point = val::object();
        point.set("x", posVec[i * 2]);
        point.set("y", posVec[i * 2 + 1]);
        point.set("size", pointSize);
        result.call<void>("push", point);
    }

    return result;
}

/**
 * Flow 벡터를 선분 데이터로 변환
 * Canvas에서 화살표 그리기용
 */
val createFlowDrawData(const val& positions, const val& flows, int count) {
    val result = val::array();

    std::vector<float> posVec = vecFromJSArray<float>(positions);
    std::vector<float> flowVec = vecFromJSArray<float>(flows);

    for (int i = 0; i < count; i++) {
        int posIdx = i * 2;
        if (posIdx + 1 >= (int)posVec.size() || posIdx + 1 >= (int)flowVec.size()) break;

        val line = val::object();
        line.set("x1", posVec[posIdx]);
        line.set("y1", posVec[posIdx + 1]);
        line.set("x2", posVec[posIdx] + flowVec[posIdx]);
        line.set("y2", posVec[posIdx + 1] + flowVec[posIdx + 1]);
        result.call<void>("push", line);
    }

    return result;
}

// ============================================================================
// Emscripten 바인딩
// ============================================================================

EMSCRIPTEN_BINDINGS(visual_odometry_module) {
    // VisualOdometryWrapper 클래스
    class_<VisualOdometryWrapper>("VisualOdometry")
        .constructor<>()
        // 설정
        .function("configure", &VisualOdometryWrapper::configure)
        .function("setCameraParams", &VisualOdometryWrapper::setCameraParams)
        // 처리
        .function("processFrame", &VisualOdometryWrapper::processFrame)
        .function("reset", &VisualOdometryWrapper::reset)
        // 특징점 데이터 (TypedArray)
        .function("getFeaturePositions", &VisualOdometryWrapper::getFeaturePositions)
        .function("getFeatureMeta", &VisualOdometryWrapper::getFeatureMeta)
        .function("getFeatureCount", &VisualOdometryWrapper::getFeatureCount)
        // 매칭 데이터
        .function("getMatches", &VisualOdometryWrapper::getMatches)
        .function("getMatchCount", &VisualOdometryWrapper::getMatchCount)
        // Flow 데이터
        .function("getFlowVectors", &VisualOdometryWrapper::getFlowVectors)
        // 포즈/행렬
        .function("getViewMatrix", &VisualOdometryWrapper::getViewMatrix)
        .function("getPose", &VisualOdometryWrapper::getPose)
        // 전체 데이터 (한 번에)
        .function("getFrameData", &VisualOdometryWrapper::getFrameData)
        // 상태
        .function("isInitialized", &VisualOdometryWrapper::isInitialized)
        .function("isTracking", &VisualOdometryWrapper::isTracking)
        .function("getFrameNumber", &VisualOdometryWrapper::getFrameNumber)
        .function("getProcessingTime", &VisualOdometryWrapper::getProcessingTime);

    // 유틸리티 함수
    function("createFeatureDrawData", &createFeatureDrawData);
    function("createFlowDrawData", &createFlowDrawData);

    // Vector 타입 등록
    register_vector<uint8_t>("VectorUint8");
    register_vector<float>("VectorFloat");
    register_vector<int>("VectorInt");
}
