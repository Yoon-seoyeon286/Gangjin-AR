/**
 * PermissionManager.js
 *
 * iOS 13+ Safari 및 Android Chrome에서 공통으로 작동하는
 * 카메라 및 센서(DeviceMotion/DeviceOrientation) 권한 관리 모듈
 *
 * iOS에서는 반드시 사용자 제스처(버튼 클릭) 내에서 권한 요청이 이루어져야 함
 */

export class PermissionManager {
    constructor() {
        // 권한 상태
        this.permissions = {
            camera: 'unknown',      // 'unknown' | 'granted' | 'denied' | 'unavailable'
            motion: 'unknown',      // DeviceMotionEvent 권한
            orientation: 'unknown'  // DeviceOrientationEvent 권한
        };

        // 디바이스 정보
        this.deviceInfo = this.detectDevice();

        // 카메라 스트림
        this.cameraStream = null;

        // 콜백
        this.onPermissionChange = null;

        console.log('[Permission] 디바이스 정보:', this.deviceInfo);
    }

    /**
     * 디바이스 및 브라우저 감지
     */
    detectDevice() {
        const ua = navigator.userAgent;

        const isIOS = /iPad|iPhone|iPod/.test(ua) && !window.MSStream;
        const isAndroid = /Android/.test(ua);
        const isSafari = /Safari/.test(ua) && !/Chrome/.test(ua) && !/CriOS/.test(ua);
        const isChrome = /Chrome/.test(ua) || /CriOS/.test(ua);

        // iOS 버전 감지
        let iosVersion = 0;
        if (isIOS) {
            const match = ua.match(/OS (\d+)_/);
            if (match) iosVersion = parseInt(match[1], 10);
        }

        // iOS 13+ Safari에서 DeviceOrientation 권한 요청 필요
        const needsMotionPermission = isIOS && iosVersion >= 13 &&
            typeof DeviceMotionEvent !== 'undefined' &&
            typeof DeviceMotionEvent.requestPermission === 'function';

        const needsOrientationPermission = isIOS && iosVersion >= 13 &&
            typeof DeviceOrientationEvent !== 'undefined' &&
            typeof DeviceOrientationEvent.requestPermission === 'function';

        return {
            isIOS,
            isAndroid,
            isSafari,
            isChrome,
            iosVersion,
            isMobile: isIOS || isAndroid,
            needsMotionPermission,
            needsOrientationPermission,
            // iOS에서 권한 요청이 필요한지 (사용자 제스처 필요)
            needsUserGesture: needsMotionPermission || needsOrientationPermission
        };
    }

    /**
     * 모든 필요한 권한을 한 번에 요청
     * 반드시 사용자 제스처(버튼 클릭) 내에서 호출해야 함!
     *
     * @returns {Promise<{camera: boolean, motion: boolean, orientation: boolean}>}
     */
    async requestAllPermissions() {
        console.log('[Permission] ===== 권한 요청 시작 =====');

        const results = {
            camera: false,
            motion: false,
            orientation: false
        };

        // 1. 카메라 권한 요청
        try {
            results.camera = await this.requestCameraPermission();
        } catch (e) {
            console.error('[Permission] 카메라 권한 에러:', e);
        }

        // 2. DeviceMotion 권한 요청 (iOS 13+)
        try {
            results.motion = await this.requestMotionPermission();
        } catch (e) {
            console.error('[Permission] Motion 권한 에러:', e);
        }

        // 3. DeviceOrientation 권한 요청 (iOS 13+)
        try {
            results.orientation = await this.requestOrientationPermission();
        } catch (e) {
            console.error('[Permission] Orientation 권한 에러:', e);
        }

        console.log('[Permission] 권한 결과:', results);
        console.log('[Permission] ===== 권한 요청 완료 =====');

        return results;
    }

    /**
     * 카메라 권한 요청 및 스트림 획득
     * @param {Object} options 카메라 옵션
     * @returns {Promise<boolean>}
     */
    async requestCameraPermission(options = {}) {
        console.log('[Permission] 카메라 권한 요청...');

        // 기본 카메라 설정
        const constraints = {
            video: {
                facingMode: options.facingMode || 'environment', // 후면 카메라
                width: { ideal: options.width || 1280 },
                height: { ideal: options.height || 720 }
            },
            audio: false
        };

        try {
            // getUserMedia 호출 - 이것 자체가 권한 요청
            this.cameraStream = await navigator.mediaDevices.getUserMedia(constraints);
            this.permissions.camera = 'granted';
            console.log('[Permission] 카메라 권한 승인됨');

            // 스트림 정보 로깅
            const track = this.cameraStream.getVideoTracks()[0];
            if (track) {
                const settings = track.getSettings();
                console.log('[Permission] 카메라 설정:', {
                    width: settings.width,
                    height: settings.height,
                    facingMode: settings.facingMode
                });
            }

            return true;

        } catch (error) {
            console.error('[Permission] 카메라 권한 실패:', error.name, error.message);

            if (error.name === 'NotAllowedError' || error.name === 'PermissionDeniedError') {
                this.permissions.camera = 'denied';
            } else if (error.name === 'NotFoundError' || error.name === 'DevicesNotFoundError') {
                this.permissions.camera = 'unavailable';
            } else {
                this.permissions.camera = 'denied';
            }

            return false;
        }
    }

    /**
     * DeviceMotionEvent 권한 요청 (iOS 13+)
     * @returns {Promise<boolean>}
     */
    async requestMotionPermission() {
        console.log('[Permission] DeviceMotion 권한 요청...');

        // iOS 13+ Safari에서만 권한 요청 필요
        if (this.deviceInfo.needsMotionPermission) {
            try {
                const permission = await DeviceMotionEvent.requestPermission();
                if (permission === 'granted') {
                    this.permissions.motion = 'granted';
                    console.log('[Permission] DeviceMotion 권한 승인됨');
                    return true;
                } else {
                    this.permissions.motion = 'denied';
                    console.log('[Permission] DeviceMotion 권한 거부됨');
                    return false;
                }
            } catch (error) {
                console.error('[Permission] DeviceMotion 권한 에러:', error);
                this.permissions.motion = 'denied';
                return false;
            }
        } else {
            // Android/데스크탑에서는 권한 요청 불필요
            // DeviceMotionEvent 지원 여부만 확인
            if (typeof DeviceMotionEvent !== 'undefined') {
                this.permissions.motion = 'granted';
                console.log('[Permission] DeviceMotion 자동 승인 (non-iOS)');
                return true;
            } else {
                this.permissions.motion = 'unavailable';
                console.log('[Permission] DeviceMotion 미지원');
                return false;
            }
        }
    }

    /**
     * DeviceOrientationEvent 권한 요청 (iOS 13+)
     * @returns {Promise<boolean>}
     */
    async requestOrientationPermission() {
        console.log('[Permission] DeviceOrientation 권한 요청...');

        // iOS 13+ Safari에서만 권한 요청 필요
        if (this.deviceInfo.needsOrientationPermission) {
            try {
                const permission = await DeviceOrientationEvent.requestPermission();
                if (permission === 'granted') {
                    this.permissions.orientation = 'granted';
                    console.log('[Permission] DeviceOrientation 권한 승인됨');
                    return true;
                } else {
                    this.permissions.orientation = 'denied';
                    console.log('[Permission] DeviceOrientation 권한 거부됨');
                    return false;
                }
            } catch (error) {
                console.error('[Permission] DeviceOrientation 권한 에러:', error);
                this.permissions.orientation = 'denied';
                return false;
            }
        } else {
            // Android/데스크탑에서는 권한 요청 불필요
            if (typeof DeviceOrientationEvent !== 'undefined') {
                this.permissions.orientation = 'granted';
                console.log('[Permission] DeviceOrientation 자동 승인 (non-iOS)');
                return true;
            } else {
                this.permissions.orientation = 'unavailable';
                console.log('[Permission] DeviceOrientation 미지원');
                return false;
            }
        }
    }

    /**
     * 카메라 스트림 가져오기
     * @returns {MediaStream|null}
     */
    getCameraStream() {
        return this.cameraStream;
    }

    /**
     * 카메라 스트림을 비디오 엘리먼트에 연결
     * @param {HTMLVideoElement} videoElement
     * @returns {Promise<boolean>}
     */
    async attachToVideo(videoElement) {
        if (!this.cameraStream) {
            console.error('[Permission] 카메라 스트림 없음');
            return false;
        }

        if (!videoElement) {
            console.error('[Permission] 비디오 엘리먼트 없음');
            return false;
        }

        try {
            videoElement.srcObject = this.cameraStream;

            // 비디오 재생 (iOS에서는 muted + playsinline 필수)
            videoElement.muted = true;
            videoElement.playsInline = true;
            videoElement.setAttribute('playsinline', '');
            videoElement.setAttribute('webkit-playsinline', '');

            await videoElement.play();

            console.log('[Permission] 비디오 연결 완료:', {
                width: videoElement.videoWidth,
                height: videoElement.videoHeight
            });

            return true;

        } catch (error) {
            console.error('[Permission] 비디오 재생 실패:', error);
            return false;
        }
    }

    /**
     * 카메라 전환 (전면 ↔ 후면)
     * @returns {Promise<boolean>}
     */
    async switchCamera() {
        console.log('[Permission] 카메라 전환 시작...');

        if (!this.cameraStream) {
            console.error('[Permission] 기존 카메라 스트림 없음');
            return false;
        }

        // 현재 facingMode 확인
        const track = this.cameraStream.getVideoTracks()[0];
        const settings = track.getSettings();
        const currentFacing = settings.facingMode || 'environment';
        const newFacing = currentFacing === 'environment' ? 'user' : 'environment';

        console.log('[Permission] 카메라 전환:', currentFacing, '→', newFacing);

        // 기존 스트림 정지
        this.stopCamera();

        // 새 카메라로 재요청
        return await this.requestCameraPermission({ facingMode: newFacing });
    }

    /**
     * 카메라 스트림 정지
     */
    stopCamera() {
        if (this.cameraStream) {
            this.cameraStream.getTracks().forEach(track => {
                track.stop();
                console.log('[Permission] 트랙 정지:', track.kind);
            });
            this.cameraStream = null;
        }
    }

    /**
     * 현재 권한 상태 반환
     */
    getPermissionStatus() {
        return { ...this.permissions };
    }

    /**
     * 모든 권한이 승인되었는지 확인
     */
    hasAllPermissions() {
        return this.permissions.camera === 'granted' &&
               (this.permissions.orientation === 'granted' ||
                this.permissions.orientation === 'unavailable');
    }

    /**
     * iOS에서 사용자 제스처가 필요한지 확인
     */
    needsUserGestureForPermission() {
        return this.deviceInfo.needsUserGesture;
    }

    /**
     * 정리
     */
    destroy() {
        this.stopCamera();
        console.log('[Permission] 정리 완료');
    }
}

// 싱글톤 인스턴스 (선택적 사용)
let instance = null;

export function getPermissionManager() {
    if (!instance) {
        instance = new PermissionManager();
    }
    return instance;
}

export default PermissionManager;
