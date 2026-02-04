#include "feature_matcher.h"
#include <iostream>

FeatureMatcher::FeatureMatcher() {
    // ORB 특징점 검출기 초기화
    orb = cv::ORB::create(1000);
    matcher = cv::BFMatcher::create(cv::NORM_HAMMING, true);
}

std::vector<cv::DMatch> FeatureMatcher::matchFeatures(
    const cv::Mat& frame1, 
    const cv::Mat& frame2,
    std::vector<cv::KeyPoint>& keypoints1,
    std::vector<cv::KeyPoint>& keypoints2
) {
    cv::Mat descriptors1, descriptors2;
    
    orb->detectAndCompute(frame1, cv::noArray(), keypoints1, descriptors1);
    orb->detectAndCompute(frame2, cv::noArray(), keypoints2, descriptors2);
    
    std::vector<cv::DMatch> matches;
    
    if (!descriptors1.empty() && !descriptors2.empty()) {
        matcher->match(descriptors1, descriptors2, matches);
        
        // 좋은 매칭만 필터링
        double min_dist = 100;
        double max_dist = 0;
        
        for (const auto& match : matches) {
            double dist = match.distance;
            if (dist < min_dist) min_dist = dist;
            if (dist > max_dist) max_dist = dist;
        }
        
        std::vector<cv::DMatch> good_matches;
        for (const auto& match : matches) {
            if (match.distance <= std::max(2.5 * min_dist, 30.0)) {
                good_matches.push_back(match);
            }
        }
        
        std::cout << "전체 매칭: " << matches.size() 
                  << ", 좋은 매칭: " << good_matches.size() << std::endl;
        
        return good_matches;
    }
    
    return matches;
}