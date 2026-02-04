export class PlaneDetector {
    constructor() {
        this.groundPlane = null;
        this.ransacThreshold = 0.01;
    }

    /**
     * Feature Points로부터 바닥 평면 찾기
     * @param {Array} features - Feature points 배열
     * @returns {Object|null} - 평면 정보 또는 null
     */
    detectPlane(features) {
        if (features.length < 4) {
            return null;
        }

        // TODO: RANSAC 알고리즘으로 평면 찾기
        // 1. 랜덤하게 3개 점 선택
        // 2. 평면 방정식 계산
        // 3. Inlier 개수 세기
        // 4. 가장 많은 Inlier를 가진 평면 선택

        console.log('평면 감지 준비 중...');
        return null;
    }

    getGroundPlane() {
        return this.groundPlane;
    }
}