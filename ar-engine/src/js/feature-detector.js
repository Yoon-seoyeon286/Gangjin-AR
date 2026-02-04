export class FeatureDetector {
    constructor() {
        this.orb = null;
        this.lastFeatures = [];
    }

    init() {
        if (typeof cv === 'undefined') {
            console.error('OpenCV.js가 로드되지 않았습니다');
            return false;
        }

        try {
            this.orb = new cv.ORB(500);
            console.log('✅ ORB Detector 초기화 완료');
            return true;
        } catch (error) {
            console.error('❌ ORB 초기화 에러:', error);
            return false;
        }
    }

    detect(video) {
        if (!this.orb || !video) return [];
        
        // 비디오 준비 상태 확인
        if (!video.videoWidth || !video.videoHeight) {
            return [];
        }

        try {
            // Canvas 생성 및 비디오 프레임 캡처
            const canvas = document.createElement('canvas');
            canvas.width = video.videoWidth;
            canvas.height = video.videoHeight;
            
            const ctx = canvas.getContext('2d');
            ctx.drawImage(video, 0, 0, canvas.width, canvas.height);

            // Canvas ImageData를 OpenCV Mat으로 변환
            const imageData = ctx.getImageData(0, 0, canvas.width, canvas.height);
            const src = cv.matFromImageData(imageData);

            // 그레이스케일 변환
            const gray = new cv.Mat();
            cv.cvtColor(src, gray, cv.COLOR_RGBA2GRAY);

            // Feature 검출
            const keypoints = new cv.KeyPointVector();
            const descriptors = new cv.Mat();
            this.orb.detectAndCompute(gray, new cv.Mat(), keypoints, descriptors);

            // KeyPoint를 배열로 변환
            const features = [];
            for (let i = 0; i < keypoints.size(); i++) {
                const kp = keypoints.get(i);
                features.push({
                    x: kp.pt.x,
                    y: kp.pt.y,
                    size: kp.size,
                    response: kp.response
                });
            }

            this.lastFeatures = features;

            // 메모리 해제
            src.delete();
            gray.delete();
            keypoints.delete();
            descriptors.delete();

            return features;

        } catch (error) {
            console.error('Feature detection 에러:', error);
            return [];
        }
    }

    getLastFeatures() {
        return this.lastFeatures;
    }

    destroy() {
        if (this.orb) {
            this.orb.delete();
        }
    }
}