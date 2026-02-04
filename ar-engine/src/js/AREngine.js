class AREngine {
    constructor() {
        this.module = null;
        this.tracker = null;
        this.isInitialized = false;

        // 캐시용 캔버스 (매 프레임 생성 방지)
        this.canvas = null;
        this.ctx = null;
    }

    async init() {
        try {
            console.log('[SLAM] AR 엔진 초기화 중...');

            // WebAssembly 모듈을 동적으로 로드
            await this.loadWasmScript('/wasm/ar-engine.js');

            if (!window.createARModule) {
                throw new Error('createARModule is not defined');
            }

            this.module = await window.createARModule();

            // ARTracker 인스턴스 생성 (SLAM 시스템 포함)
            this.tracker = new this.module.ARTracker();

            this.isInitialized = true;
            console.log('[SLAM] AR 엔진 초기화 완료!');

            return true;
        } catch (error) {
            console.error('[SLAM] AR 엔진 초기화 실패:', error);
            return false;
        }
    }

    loadWasmScript(src) {
        return new Promise((resolve, reject) => {
            if (window.createARModule) {
                resolve();
                return;
            }

            const script = document.createElement('script');
            script.src = src;
            script.async = true;
            script.onload = () => resolve();
            script.onerror = () => reject(new Error(`Failed to load script: ${src}`));
            document.head.appendChild(script);
        });
    }

    processFrame(videoElement) {
        if (!this.isInitialized || !this.tracker) {
            return false;
        }

        try {
            const width = videoElement.videoWidth;
            const height = videoElement.videoHeight;

            if (width === 0 || height === 0) return false;

            // 캔버스 재사용
            if (!this.canvas || this.canvas.width !== width || this.canvas.height !== height) {
                this.canvas = document.createElement('canvas');
                this.canvas.width = width;
                this.canvas.height = height;
                this.ctx = this.canvas.getContext('2d');
            }

            this.ctx.drawImage(videoElement, 0, 0);
            const imageData = this.ctx.getImageData(0, 0, width, height);

            const success = this.tracker.processFrame(width, height, imageData.data);

            return success;
        } catch (error) {
            console.error('[SLAM] 프레임 처리 오류:', error);
            return false;
        }
    }

    getViewMatrix() {
        if (!this.tracker) return null;

        try {
            const matrixArray = this.tracker.getViewMatrix();
            const matrix = [];
            for (let i = 0; i < 16; i++) {
                matrix.push(matrixArray[i]);
            }
            return matrix;
        } catch (error) {
            console.error('[SLAM] View Matrix 가져오기 실패:', error);
            return null;
        }
    }

    getProjectionMatrix(width, height) {
        if (!this.tracker) return null;

        try {
            const matrixArray = this.tracker.getProjectionMatrix(width, height);
            const matrix = [];
            for (let i = 0; i < 16; i++) {
                matrix.push(matrixArray[i]);
            }
            return matrix;
        } catch (error) {
            console.error('[SLAM] Projection Matrix 가져오기 실패:', error);
            return null;
        }
    }

    // SLAM 상태 정보
    isSlamInitialized() {
        if (!this.tracker) return false;
        try {
            return this.tracker.isInitialized();
        } catch {
            return false;
        }
    }

    isSlamTracking() {
        if (!this.tracker) return false;
        try {
            return this.tracker.isTracking();
        } catch {
            return false;
        }
    }

    getMapPointCount() {
        if (!this.tracker) return 0;
        try {
            return this.tracker.getMapPointCount();
        } catch {
            return 0;
        }
    }

    getKeyFrameCount() {
        if (!this.tracker) return 0;
        try {
            return this.tracker.getKeyFrameCount();
        } catch {
            return 0;
        }
    }

    destroy() {
        if (this.tracker) {
            this.tracker.delete();
            this.tracker = null;
        }
        this.isInitialized = false;
        this.canvas = null;
        this.ctx = null;
        console.log('[SLAM] AR 엔진 종료');
    }
}

export default AREngine;
