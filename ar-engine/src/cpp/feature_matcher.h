#ifndef FEATURE_MATCHER_H
#define FEATURE_MATCHER_H

#include <opencv2/opencv.hpp>
#include <vector>

class FeatureMatcher {
public:
    FeatureMatcher();
    
    // 두 프레임 간의 특징점을 매칭하는 함수
    std::vector<cv::DMatch> matchFeatures(
        const cv::Mat& frame1, 
        const cv::Mat& frame2,
        std::vector<cv::KeyPoint>& keypoints1,
        std::vector<cv::KeyPoint>& keypoints2
    );

private:
    cv::Ptr<cv::ORB> orb;              // 특징점 검출기
    cv::Ptr<cv::BFMatcher> matcher;    // 특징점 매칭기
};

#endif