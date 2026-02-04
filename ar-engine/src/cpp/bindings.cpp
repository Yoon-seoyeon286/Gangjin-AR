#include <emscripten/bind.h>
#include <emscripten/val.h>
#include "ar_tracker.h"
#include "slam_system.h"
#include "plane_detector.h"
#include "image_target.h"
#include <opencv2/opencv.hpp>

using namespace emscripten;

cv::Mat imageDataToMat(int width, int height, const val& data) {
    std::vector<uint8_t> buffer = vecFromJSArray<uint8_t>(data);
    cv::Mat rgba(height, width, CV_8UC4, buffer.data());
    cv::Mat gray;
    cv::cvtColor(rgba, gray, cv::COLOR_RGBA2GRAY);
    return gray;
}

cv::Mat imageDataToMatColor(int width, int height, const val& data) {
    std::vector<uint8_t> buffer = vecFromJSArray<uint8_t>(data);
    cv::Mat rgba(height, width, CV_8UC4, buffer.data());
    return rgba.clone();
}

val matToArray(const cv::Mat& mat) {
    val result = val::array();
    for (int i = 0; i < mat.rows; i++) {
        for (int j = 0; j < mat.cols; j++) {
            result.call<void>("push", mat.at<double>(i, j));
        }
    }
    return result;
}

class ARTrackerWrapper {
public:
    ARTrackerWrapper() {
        tracker = new ARTracker();
    }

    ~ARTrackerWrapper() {
        delete tracker;
    }

    bool processFrame(int width, int height, const val& imageData) {
        cv::Mat frame = imageDataToMat(width, height, imageData);
        return tracker->processFrame(frame);
    }

    val getViewMatrix() {
        cv::Mat viewMat = tracker->getViewMatrix();
        return matToArray(viewMat);
    }

    val getProjectionMatrix(int width, int height) {
        cv::Mat projMat = tracker->getProjectionMatrix(width, height);
        return matToArray(projMat);
    }

    // SLAM 상태 정보
    bool isInitialized() {
        return tracker->isInitialized();
    }

    bool isTracking() {
        return tracker->isTracking();
    }

    int getMapPointCount() {
        return tracker->getMapPointCount();
    }

    int getKeyFrameCount() {
        return tracker->getKeyFrameCount();
    }

    // ==================== 평면 감지 ====================

    int getPlaneCount() {
        auto slam = tracker->getSLAM();
        if (!slam) return 0;
        return slam->getDetectedPlanes().size();
    }

    val getDetectedPlanes() {
        val result = val::array();
        auto slam = tracker->getSLAM();
        if (!slam) return result;

        auto planes = slam->getDetectedPlanes();
        for (const auto& plane : planes) {
            val planeObj = val::object();
            planeObj.set("id", plane.id);
            planeObj.set("isHorizontal", plane.isHorizontal);
            planeObj.set("confidence", plane.confidence);
            planeObj.set("width", plane.width);
            planeObj.set("height", plane.height);

            // 중심점
            val center = val::array();
            center.call<void>("push", plane.center.x);
            center.call<void>("push", plane.center.y);
            center.call<void>("push", plane.center.z);
            planeObj.set("center", center);

            // 법선 벡터
            val normal = val::array();
            normal.call<void>("push", plane.normal.x);
            normal.call<void>("push", plane.normal.y);
            normal.call<void>("push", plane.normal.z);
            planeObj.set("normal", normal);

            // 4개 코너
            val corners = val::array();
            for (const auto& corner : plane.corners) {
                val c = val::array();
                c.call<void>("push", corner.x);
                c.call<void>("push", corner.y);
                c.call<void>("push", corner.z);
                corners.call<void>("push", c);
            }
            planeObj.set("corners", corners);

            result.call<void>("push", planeObj);
        }
        return result;
    }

    // ==================== 이미지 타겟 ====================

    int addImageTarget(int width, int height, const val& imageData,
                       const std::string& name, float widthMeters) {
        auto slam = tracker->getSLAM();
        if (!slam) return -1;

        cv::Mat image = imageDataToMat(width, height, imageData);
        return slam->addImageTarget(image, name, widthMeters);
    }

    int getTargetCount() {
        auto slam = tracker->getSLAM();
        if (!slam) return 0;
        auto tracker = slam->getImageTargetTracker();
        if (!tracker) return 0;
        return tracker->getTargetCount();
    }

    val getDetectedTargets() {
        val result = val::array();
        auto slam = tracker->getSLAM();
        if (!slam) return result;

        auto targets = slam->getDetectedTargets();
        for (const auto& target : targets) {
            val targetObj = val::object();
            targetObj.set("id", target.targetId);
            targetObj.set("name", target.name);
            targetObj.set("confidence", target.confidence);
            targetObj.set("isTracking", target.isTracking);

            // 포즈 행렬 (4x4)
            val pose = val::array();
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    pose.call<void>("push", target.pose.at<double>(i, j));
                }
            }
            targetObj.set("pose", pose);

            // 화면상 4개 코너
            val corners = val::array();
            for (const auto& corner : target.corners) {
                val c = val::array();
                c.call<void>("push", corner.x);
                c.call<void>("push", corner.y);
                corners.call<void>("push", c);
            }
            targetObj.set("corners", corners);

            result.call<void>("push", targetObj);
        }
        return result;
    }

private:
    ARTracker* tracker;
};

EMSCRIPTEN_BINDINGS(ar_module) {
    class_<ARTrackerWrapper>("ARTracker")
        .constructor<>()
        .function("processFrame", &ARTrackerWrapper::processFrame)
        .function("getViewMatrix", &ARTrackerWrapper::getViewMatrix)
        .function("getProjectionMatrix", &ARTrackerWrapper::getProjectionMatrix)
        .function("isInitialized", &ARTrackerWrapper::isInitialized)
        .function("isTracking", &ARTrackerWrapper::isTracking)
        .function("getMapPointCount", &ARTrackerWrapper::getMapPointCount)
        .function("getKeyFrameCount", &ARTrackerWrapper::getKeyFrameCount)
        // 평면 감지
        .function("getPlaneCount", &ARTrackerWrapper::getPlaneCount)
        .function("getDetectedPlanes", &ARTrackerWrapper::getDetectedPlanes)
        // 이미지 타겟
        .function("addImageTarget", &ARTrackerWrapper::addImageTarget)
        .function("getTargetCount", &ARTrackerWrapper::getTargetCount)
        .function("getDetectedTargets", &ARTrackerWrapper::getDetectedTargets);

    register_vector<uint8_t>("VectorUint8");
}
