import * as THREE from 'three';

/**
 * AR 디스플레이 시스템
 *
 * 두 가지 방식 지원:
 * 1. Video See-through: 스마트폰 (카메라 영상 위에 3D 렌더링)
 * 2. Optical See-through: AR 글래스 (WebXR 기반 투명 렌더링)
 */

// 디스플레이 모드
export const DisplayMode = {
    VIDEO_SEE_THROUGH: 'video',      // 스마트폰, 태블릿
    OPTICAL_SEE_THROUGH: 'optical',  // AR 글래스, HoloLens
    AUTO: 'auto'                      // 자동 감지
};

export class DisplaySystem {
    constructor() {
        this.mode = DisplayMode.VIDEO_SEE_THROUGH;
        this.renderer = null;
        this.camera = null;
        this.scene = null;

        // Video See-through
        this.videoElement = null;
        this.videoTexture = null;

        // Optical See-through (WebXR)
        this.xrSession = null;
        this.xrRefSpace = null;
        this.isXRSupported = false;

        // 설정
        this.config = {
            fov: 70,
            near: 0.01,
            far: 1000,
            antialias: true,
            alpha: true
        };

        console.log('[Display] 시스템 초기화');
    }

    /**
     * 디스플레이 시스템 초기화
     */
    async init(container, mode = DisplayMode.AUTO) {
        // WebXR 지원 확인
        await this.checkXRSupport();

        // 모드 결정
        if (mode === DisplayMode.AUTO) {
            this.mode = this.isXRSupported ?
                DisplayMode.OPTICAL_SEE_THROUGH :
                DisplayMode.VIDEO_SEE_THROUGH;
        } else {
            this.mode = mode;
        }

        console.log('[Display] 모드:', this.mode);

        // Three.js 기본 설정
        this.setupRenderer(container);
        this.setupCamera();
        this.setupScene();

        // 모드별 초기화
        if (this.mode === DisplayMode.VIDEO_SEE_THROUGH) {
            await this.initVideoSeeThrough();
        } else {
            await this.initOpticalSeeThrough();
        }

        // 리사이즈 핸들러
        window.addEventListener('resize', () => this.onResize());

        return true;
    }

    /**
     * WebXR 지원 확인
     */
    async checkXRSupport() {
        if ('xr' in navigator) {
            try {
                this.isXRSupported = await navigator.xr.isSessionSupported('immersive-ar');
                console.log('[Display] WebXR AR 지원:', this.isXRSupported);
            } catch (e) {
                this.isXRSupported = false;
            }
        }
    }

    /**
     * Three.js 렌더러 설정
     */
    setupRenderer(container) {
        this.renderer = new THREE.WebGLRenderer({
            antialias: this.config.antialias,
            alpha: this.config.alpha,
            preserveDrawingBuffer: true
        });

        this.renderer.setPixelRatio(window.devicePixelRatio);
        this.renderer.setSize(window.innerWidth, window.innerHeight);
        this.renderer.setClearColor(0x000000, 0);
        this.renderer.outputColorSpace = THREE.SRGBColorSpace;

        // Optical See-through용 XR 활성화
        if (this.isXRSupported) {
            this.renderer.xr.enabled = true;
        }

        container.appendChild(this.renderer.domElement);
        console.log('[Display] 렌더러 생성');
    }

    /**
     * 카메라 설정
     */
    setupCamera() {
        this.camera = new THREE.PerspectiveCamera(
            this.config.fov,
            window.innerWidth / window.innerHeight,
            this.config.near,
            this.config.far
        );
        console.log('[Display] 카메라 생성');
    }

    /**
     * 씬 설정
     */
    setupScene() {
        this.scene = new THREE.Scene();

        // 기본 조명
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
        this.scene.add(ambientLight);

        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
        directionalLight.position.set(1, 2, 1);
        this.scene.add(directionalLight);

        console.log('[Display] 씬 생성');
    }

    // ==================== Video See-through ====================

    /**
     * Video See-through 초기화
     */
    async initVideoSeeThrough() {
        console.log('[Display] Video See-through 초기화');

        // 비디오 엘리먼트 찾기 또는 생성
        this.videoElement = document.getElementById('video-background');
        if (!this.videoElement) {
            this.videoElement = document.createElement('video');
            this.videoElement.id = 'video-background';
            this.videoElement.setAttribute('playsinline', '');
            this.videoElement.setAttribute('autoplay', '');
            this.videoElement.style.cssText = `
                position: absolute;
                top: 0;
                left: 0;
                width: 100%;
                height: 100%;
                object-fit: cover;
                z-index: 0;
            `;
            document.body.insertBefore(this.videoElement, document.body.firstChild);
        }

        // 카메라 스트림 시작
        await this.startCameraStream();

        return true;
    }

    /**
     * 카메라 스트림 시작
     */
    async startCameraStream(facingMode = 'environment') {
        try {
            const constraints = {
                video: {
                    facingMode: facingMode,
                    width: { ideal: 1280 },
                    height: { ideal: 720 }
                }
            };

            const stream = await navigator.mediaDevices.getUserMedia(constraints);
            this.videoElement.srcObject = stream;
            await this.videoElement.play();

            console.log('[Display] 카메라 시작:',
                this.videoElement.videoWidth, 'x',
                this.videoElement.videoHeight);

            // 카메라 FOV 업데이트
            this.updateCameraFOV();

            return true;
        } catch (error) {
            console.error('[Display] 카메라 에러:', error);
            return false;
        }
    }

    /**
     * 카메라 전환 (전면/후면)
     */
    async switchCamera() {
        if (this.mode !== DisplayMode.VIDEO_SEE_THROUGH) return;

        // 현재 스트림 정지
        if (this.videoElement.srcObject) {
            this.videoElement.srcObject.getTracks().forEach(t => t.stop());
        }

        // 현재 facingMode 확인
        const currentTrack = this.videoElement.srcObject?.getVideoTracks()[0];
        const currentFacing = currentTrack?.getSettings().facingMode;
        const newFacing = currentFacing === 'environment' ? 'user' : 'environment';

        await this.startCameraStream(newFacing);
    }

    /**
     * 비디오 화면비에 맞춰 카메라 FOV 업데이트
     */
    updateCameraFOV() {
        if (!this.videoElement || !this.camera) return;

        const videoAspect = this.videoElement.videoWidth / this.videoElement.videoHeight;
        const screenAspect = window.innerWidth / window.innerHeight;

        // 비디오와 화면 비율이 다를 경우 FOV 조정
        if (videoAspect > screenAspect) {
            // 비디오가 더 넓음 - 좌우 잘림
            this.camera.fov = this.config.fov;
        } else {
            // 비디오가 더 좁음 - 상하 잘림
            this.camera.fov = this.config.fov * (screenAspect / videoAspect);
        }
        this.camera.updateProjectionMatrix();
    }

    // ==================== Optical See-through (WebXR) ====================

    /**
     * Optical See-through 초기화 (WebXR)
     */
    async initOpticalSeeThrough() {
        console.log('[Display] Optical See-through 초기화 (WebXR)');

        if (!this.isXRSupported) {
            console.warn('[Display] WebXR AR 미지원, Video See-through로 폴백');
            this.mode = DisplayMode.VIDEO_SEE_THROUGH;
            return this.initVideoSeeThrough();
        }

        // XR 세션 시작 버튼 추가
        this.createXRButton();

        return true;
    }

    /**
     * WebXR 세션 시작 버튼 생성
     */
    createXRButton() {
        const button = document.createElement('button');
        button.id = 'xr-button';
        button.textContent = 'AR 시작';
        button.style.cssText = `
            position: fixed;
            bottom: 80px;
            left: 50%;
            transform: translateX(-50%);
            padding: 15px 30px;
            background: linear-gradient(135deg, #4da6ff, #80c0ff);
            border: none;
            border-radius: 25px;
            color: #000;
            font-size: 18px;
            font-weight: bold;
            cursor: pointer;
            z-index: 1000;
        `;

        button.addEventListener('click', () => this.startXRSession());
        document.body.appendChild(button);
    }

    /**
     * WebXR 세션 시작
     */
    async startXRSession() {
        try {
            const sessionInit = {
                requiredFeatures: ['local-floor'],
                optionalFeatures: ['dom-overlay', 'hit-test', 'plane-detection']
            };

            this.xrSession = await navigator.xr.requestSession('immersive-ar', sessionInit);

            this.xrSession.addEventListener('end', () => this.onXRSessionEnd());

            await this.renderer.xr.setSession(this.xrSession);

            this.xrRefSpace = await this.xrSession.requestReferenceSpace('local-floor');

            // XR 버튼 숨기기
            const button = document.getElementById('xr-button');
            if (button) button.style.display = 'none';

            console.log('[Display] WebXR 세션 시작');
        } catch (error) {
            console.error('[Display] WebXR 세션 시작 실패:', error);
        }
    }

    /**
     * WebXR 세션 종료 핸들러
     */
    onXRSessionEnd() {
        this.xrSession = null;
        console.log('[Display] WebXR 세션 종료');

        // XR 버튼 다시 표시
        const button = document.getElementById('xr-button');
        if (button) button.style.display = 'block';
    }

    // ==================== 공통 메서드 ====================

    /**
     * 화면 리사이즈 핸들러
     */
    onResize() {
        const width = window.innerWidth;
        const height = window.innerHeight;

        this.camera.aspect = width / height;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(width, height);

        if (this.mode === DisplayMode.VIDEO_SEE_THROUGH) {
            this.updateCameraFOV();
        }
    }

    /**
     * 렌더링
     */
    render() {
        if (this.xrSession) {
            // WebXR 모드에서는 자동 렌더링
            return;
        }

        this.renderer.render(this.scene, this.camera);
    }

    /**
     * 카메라 포즈 업데이트
     */
    updateCameraPose(position, quaternion) {
        if (this.xrSession) {
            // WebXR 모드에서는 XR이 카메라를 제어
            return;
        }

        this.camera.position.copy(position);
        this.camera.quaternion.copy(quaternion);
    }

    /**
     * 현재 모드 반환
     */
    getMode() {
        return this.mode;
    }

    /**
     * WebXR 세션 활성 여부
     */
    isXRActive() {
        return this.xrSession !== null;
    }

    /**
     * 비디오 엘리먼트 반환
     */
    getVideoElement() {
        return this.videoElement;
    }

    /**
     * Three.js 객체 반환
     */
    getRenderer() { return this.renderer; }
    getCamera() { return this.camera; }
    getScene() { return this.scene; }

    /**
     * 정리
     */
    destroy() {
        // 비디오 스트림 정지
        if (this.videoElement?.srcObject) {
            this.videoElement.srcObject.getTracks().forEach(t => t.stop());
        }

        // XR 세션 종료
        if (this.xrSession) {
            this.xrSession.end();
        }

        // 렌더러 정리
        if (this.renderer) {
            this.renderer.dispose();
        }

        console.log('[Display] 시스템 종료');
    }
}

export default DisplaySystem;
