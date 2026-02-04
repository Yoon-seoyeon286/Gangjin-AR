/**
 * VisualOdometry.js
 *
 * C++ VisualOdometry 클래스의 JavaScript 래퍼
 * FAST 특징점 추출 + Optical Flow + 포즈 추정
 *
 * 사용법:
 * ```javascript
 * import { VisualOdometry } from './VisualOdometry.js';
 *
 * const vo = new VisualOdometry();
 * await vo.init();
 *
 * // 프레임 처리
 * const result = vo.processFrame(imageData);
 * console.log('특징점:', result.featureCount);
 * console.log('View Matrix:', result.viewMatrix);
 * ```
 */

let wasmModule = null;
let wasmLoaded = false;

/**
 * Wasm 모듈 로드
 */
async function loadWasmModule() {
    if (wasmLoaded) return wasmModule;

    try {
        // 스크립트 동적 로드
        if (typeof createVOModule === 'undefined') {
            await new Promise((resolve, reject) => {
                const script = document.createElement('script');
                script.src = '/wasm/visual-odometry.js';
                script.onload = resolve;
                script.onerror = reject;
                document.head.appendChild(script);
            });
        }

        wasmModule = await createVOModule();
        wasmLoaded = true;
        console.log('[VO] WebAssembly 모듈 로드 완료');
        return wasmModule;

    } catch (error) {
        console.error('[VO] Wasm 로드 실패:', error);
        throw error;
    }
}

/**
 * VisualOdometry 클래스
 */
export class VisualOdometry {
    constructor() {
        this.vo = null;
        this.initialized = false;
        this.canvas = null;
        this.ctx = null;

        // 설정
        this.config = {
            fastThreshold: 20,
            maxFeatures: 500,
            focalLength: 800,
            cx: 640,
            cy: 360
        };

        // 캐시된 결과
        this.lastResult = null;
    }

    /**
     * 초기화
     * @param {Object} config 설정 옵션
     */
    async init(config = {}) {
        // 설정 병합
        this.config = { ...this.config, ...config };

        // Wasm 모듈 로드
        const module = await loadWasmModule();

        // C++ 객체 생성
        this.vo = new module.VisualOdometry();

        // 설정 적용
        this.vo.configure(
            this.config.fastThreshold,
            this.config.maxFeatures,
            this.config.focalLength,
            this.config.cx,
            this.config.cy
        );

        this.initialized = true;
        console.log('[VO] 초기화 완료:', this.config);
    }

    /**
     * 카메라 파라미터 설정
     * @param {number} fx 초점 거리 X
     * @param {number} fy 초점 거리 Y
     * @param {number} cx 주점 X
     * @param {number} cy 주점 Y
     */
    setCameraParams(fx, fy, cx, cy) {
        if (!this.vo) return;
        this.vo.setCameraParams(fx, fy, cx, cy);
    }

    /**
     * 비디오 해상도에 맞게 카메라 파라미터 자동 설정
     * @param {number} width 비디오 너비
     * @param {number} height 비디오 높이
     * @param {number} fovDegrees 시야각 (기본 60도)
     */
    autoConfigureCamera(width, height, fovDegrees = 60) {
        const fovRad = (fovDegrees * Math.PI) / 180;
        const fx = width / (2 * Math.tan(fovRad / 2));
        const fy = fx;
        const cx = width / 2;
        const cy = height / 2;

        this.setCameraParams(fx, fy, cx, cy);
        console.log('[VO] 카메라 자동 설정:', { fx, fy, cx, cy });
    }

    /**
     * 프레임 처리
     * @param {ImageData} imageData Canvas ImageData 객체
     * @returns {Object} 처리 결과
     */
    processFrame(imageData) {
        if (!this.vo || !this.initialized) {
            console.warn('[VO] 초기화되지 않음');
            return null;
        }

        const { width, height, data } = imageData;

        // C++ 처리
        const success = this.vo.processFrame(width, height, data);

        if (!success) {
            return null;
        }

        // 결과 가져오기 (한 번의 호출로 모든 데이터)
        const result = this.vo.getFrameData();

        // TypedArray를 일반 배열로 변환 (필요시)
        this.lastResult = {
            // 특징점
            featurePositions: result.featurePositions,  // Float32Array
            featureMeta: result.featureMeta,            // Float32Array
            featureCount: result.featureCount,

            // 매칭
            matches: result.matches,                     // Int32Array
            matchCount: result.matchCount,

            // Optical Flow
            flowVectors: result.flowVectors,             // Float32Array

            // 포즈
            pose: {
                quaternion: {
                    x: result.pose.qx,
                    y: result.pose.qy,
                    z: result.pose.qz,
                    w: result.pose.qw
                },
                position: {
                    x: result.pose.tx,
                    y: result.pose.ty,
                    z: result.pose.tz
                },
                confidence: result.pose.confidence,
                valid: result.pose.valid
            },

            // View Matrix (Float32Array, 16 elements, column-major)
            viewMatrix: result.viewMatrix,

            // 상태
            initialized: result.initialized,
            tracking: result.tracking,
            frameNumber: result.frameNumber,
            processingTimeMs: result.processingTimeMs
        };

        return this.lastResult;
    }

    /**
     * 비디오에서 직접 프레임 처리
     * @param {HTMLVideoElement} video 비디오 엘리먼트
     * @returns {Object} 처리 결과
     */
    processVideo(video) {
        if (!video || video.videoWidth === 0) {
            return null;
        }

        // Canvas가 없으면 생성
        if (!this.canvas) {
            this.canvas = document.createElement('canvas');
            this.ctx = this.canvas.getContext('2d', { willReadFrequently: true });
        }

        // Canvas 크기 설정
        if (this.canvas.width !== video.videoWidth ||
            this.canvas.height !== video.videoHeight) {
            this.canvas.width = video.videoWidth;
            this.canvas.height = video.videoHeight;

            // 카메라 파라미터 자동 설정
            this.autoConfigureCamera(video.videoWidth, video.videoHeight);
        }

        // 비디오 프레임을 Canvas에 그리기
        this.ctx.drawImage(video, 0, 0);

        // ImageData 추출
        const imageData = this.ctx.getImageData(0, 0, this.canvas.width, this.canvas.height);

        // 처리
        return this.processFrame(imageData);
    }

    /**
     * 리셋
     */
    reset() {
        if (this.vo) {
            this.vo.reset();
        }
        this.lastResult = null;
    }

    /**
     * 특징점을 Canvas에 그리기
     * @param {CanvasRenderingContext2D} ctx Canvas 2D Context
     * @param {Object} options 그리기 옵션
     */
    drawFeatures(ctx, options = {}) {
        if (!this.lastResult) return;

        const {
            color = '#4da6ff',
            size = 4,
            showAge = false,
            ageColors = ['#4da6ff', '#ffff00', '#ff8800', '#ff0000']
        } = options;

        const { featurePositions, featureMeta, featureCount } = this.lastResult;

        ctx.save();

        for (let i = 0; i < featureCount; i++) {
            const x = featurePositions[i * 2];
            const y = featurePositions[i * 2 + 1];

            let pointColor = color;

            // 나이에 따른 색상 (오래 추적된 특징점일수록 빨간색)
            if (showAge && featureMeta) {
                const age = featureMeta[i * 4 + 3];
                const colorIdx = Math.min(Math.floor(age / 10), ageColors.length - 1);
                pointColor = ageColors[colorIdx];
            }

            ctx.beginPath();
            ctx.arc(x, y, size, 0, Math.PI * 2);
            ctx.fillStyle = pointColor;
            ctx.fill();
        }

        ctx.restore();
    }

    /**
     * Optical Flow 벡터를 Canvas에 그리기
     * @param {CanvasRenderingContext2D} ctx Canvas 2D Context
     * @param {Object} options 그리기 옵션
     */
    drawFlow(ctx, options = {}) {
        if (!this.lastResult) return;

        const {
            color = '#00aaff',
            lineWidth = 1,
            scale = 3  // Flow 벡터 스케일
        } = options;

        const { featurePositions, flowVectors, featureCount } = this.lastResult;

        if (!flowVectors || flowVectors.length === 0) return;

        ctx.save();
        ctx.strokeStyle = color;
        ctx.lineWidth = lineWidth;

        for (let i = 0; i < featureCount && i * 2 + 1 < flowVectors.length; i++) {
            const x = featurePositions[i * 2];
            const y = featurePositions[i * 2 + 1];
            const dx = flowVectors[i * 2] * scale;
            const dy = flowVectors[i * 2 + 1] * scale;

            ctx.beginPath();
            ctx.moveTo(x, y);
            ctx.lineTo(x + dx, y + dy);
            ctx.stroke();
        }

        ctx.restore();
    }

    /**
     * View Matrix를 Three.js Matrix4로 변환
     * @param {THREE.Matrix4} matrix 대상 Matrix4 (선택)
     * @returns {THREE.Matrix4}
     */
    toThreeMatrix4(matrix = null) {
        if (!this.lastResult || !this.lastResult.viewMatrix) {
            return null;
        }

        if (typeof THREE === 'undefined') {
            console.warn('[VO] Three.js가 로드되지 않음');
            return null;
        }

        const mat = matrix || new THREE.Matrix4();
        mat.fromArray(this.lastResult.viewMatrix);
        return mat;
    }

    /**
     * 포즈를 Three.js Quaternion + Vector3로 변환
     * @returns {{quaternion: THREE.Quaternion, position: THREE.Vector3}}
     */
    toThreePose() {
        if (!this.lastResult || !this.lastResult.pose.valid) {
            return null;
        }

        if (typeof THREE === 'undefined') {
            console.warn('[VO] Three.js가 로드되지 않음');
            return null;
        }

        const { pose } = this.lastResult;

        return {
            quaternion: new THREE.Quaternion(
                pose.quaternion.x,
                pose.quaternion.y,
                pose.quaternion.z,
                pose.quaternion.w
            ),
            position: new THREE.Vector3(
                pose.position.x,
                pose.position.y,
                pose.position.z
            )
        };
    }

    /**
     * 상태 조회
     */
    isInitialized() {
        return this.lastResult?.initialized || false;
    }

    isTracking() {
        return this.lastResult?.tracking || false;
    }

    getFeatureCount() {
        return this.lastResult?.featureCount || 0;
    }

    getProcessingTime() {
        return this.lastResult?.processingTimeMs || 0;
    }

    /**
     * 정리
     */
    destroy() {
        if (this.vo) {
            this.vo.delete();
            this.vo = null;
        }
        this.canvas = null;
        this.ctx = null;
        this.initialized = false;
        console.log('[VO] 정리 완료');
    }
}

/**
 * 싱글톤 인스턴스 (선택적 사용)
 */
let instance = null;

export async function getVisualOdometry(config = {}) {
    if (!instance) {
        instance = new VisualOdometry();
        await instance.init(config);
    }
    return instance;
}

export default VisualOdometry;
