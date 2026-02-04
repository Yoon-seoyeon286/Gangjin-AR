import * as THREE from 'three';

/**
 * 센서 퓨전 시스템
 *
 * 여러 센서를 결합하여 안정적인 6DoF 추적:
 * - 자이로스코프: 각속도 (빠른 회전 추적)
 * - 가속도계: 선형 가속도 (중력 방향, 이동)
 * - 지자기계: 자북 방향 (절대 방향)
 * - Visual Odometry: 카메라 기반 위치 추적
 *
 * 필터:
 * - 상보 필터: 자이로 드리프트 보정
 * - 칼만 필터: 노이즈 제거 및 예측
 */

export class SensorFusion {
    constructor() {
        // === 센서 데이터 ===
        this.gyroscope = { alpha: 0, beta: 0, gamma: 0 };       // 자이로스코프 (deg)
        this.accelerometer = { x: 0, y: 0, z: 0 };              // 가속도계 (m/s²)
        this.magnetometer = { x: 0, y: 0, z: 0 };               // 지자기계 (μT)

        // === 초기 기준값 ===
        this.initialOrientation = null;
        this.initialAccel = null;
        this.calibrated = false;

        // === 필터링된 결과 ===
        this.orientation = new THREE.Quaternion();      // 최종 회전
        this.position = new THREE.Vector3();            // 최종 위치
        this.velocity = new THREE.Vector3();            // 속도

        // === 칼만 필터 상태 ===
        this.kalman = {
            // 회전용
            orientation: {
                estimate: new THREE.Quaternion(),
                errorCovariance: 1.0,
                processNoise: 0.001,
                measurementNoise: 0.1
            },
            // 위치용
            position: {
                estimate: new THREE.Vector3(),
                velocity: new THREE.Vector3(),
                errorCovariance: 1.0,
                processNoise: 0.01,
                measurementNoise: 0.5
            }
        };

        // === 상보 필터 설정 ===
        this.complementaryAlpha = 0.98;  // 자이로 비중 (높을수록 자이로 신뢰)

        // === 타임스탬프 ===
        this.lastTimestamp = 0;
        this.deltaTime = 0;

        // === 센서 사용 가능 여부 ===
        this.sensorsAvailable = {
            deviceOrientation: false,
            deviceMotion: false,
            magnetometer: false
        };

        // === Visual Odometry 데이터 ===
        this.voPosition = new THREE.Vector3();
        this.voEnabled = false;

        // === 저주파 필터 (노이즈 제거) ===
        this.lowPassFilter = {
            alpha: 0.8,  // 필터 강도
            accel: { x: 0, y: 0, z: 0 },
            gyro: { alpha: 0, beta: 0, gamma: 0 }
        };

        console.log('[SensorFusion] 초기화');
    }

    /**
     * 센서 초기화 및 이벤트 바인딩
     */
    async init() {
        console.log('[SensorFusion] 센서 초기화 시작...');

        // DeviceOrientation (자이로스코프)
        await this.initDeviceOrientation();

        // DeviceMotion (가속도계)
        await this.initDeviceMotion();

        // Magnetometer (지자기계) - Generic Sensor API
        await this.initMagnetometer();

        console.log('[SensorFusion] 센서 상태:', this.sensorsAvailable);

        return this.sensorsAvailable.deviceOrientation;
    }

    /**
     * DeviceOrientation 초기화 (자이로스코프)
     */
    async initDeviceOrientation() {
        if (!window.DeviceOrientationEvent) {
            console.warn('[SensorFusion] DeviceOrientation 미지원');
            return;
        }

        // iOS 권한 요청
        if (typeof DeviceOrientationEvent.requestPermission === 'function') {
            try {
                const permission = await DeviceOrientationEvent.requestPermission();
                if (permission !== 'granted') {
                    console.warn('[SensorFusion] DeviceOrientation 권한 거부');
                    return;
                }
            } catch (e) {
                console.warn('[SensorFusion] DeviceOrientation 권한 요청 실패:', e);
                return;
            }
        }

        window.addEventListener('deviceorientation', (e) => this.onDeviceOrientation(e), true);
        this.sensorsAvailable.deviceOrientation = true;
        console.log('[SensorFusion] DeviceOrientation 활성화');
    }

    /**
     * DeviceMotion 초기화 (가속도계)
     */
    async initDeviceMotion() {
        if (!window.DeviceMotionEvent) {
            console.warn('[SensorFusion] DeviceMotion 미지원');
            return;
        }

        // iOS 권한 요청
        if (typeof DeviceMotionEvent.requestPermission === 'function') {
            try {
                const permission = await DeviceMotionEvent.requestPermission();
                if (permission !== 'granted') {
                    console.warn('[SensorFusion] DeviceMotion 권한 거부');
                    return;
                }
            } catch (e) {
                console.warn('[SensorFusion] DeviceMotion 권한 요청 실패:', e);
                return;
            }
        }

        window.addEventListener('devicemotion', (e) => this.onDeviceMotion(e), true);
        this.sensorsAvailable.deviceMotion = true;
        console.log('[SensorFusion] DeviceMotion 활성화');
    }

    /**
     * Magnetometer 초기화 (지자기계)
     */
    async initMagnetometer() {
        if (!('Magnetometer' in window)) {
            console.warn('[SensorFusion] Magnetometer API 미지원');
            return;
        }

        try {
            const sensor = new Magnetometer({ frequency: 60 });
            sensor.addEventListener('reading', () => {
                this.magnetometer.x = sensor.x;
                this.magnetometer.y = sensor.y;
                this.magnetometer.z = sensor.z;
            });
            sensor.addEventListener('error', (e) => {
                console.warn('[SensorFusion] Magnetometer 에러:', e.error.message);
            });
            sensor.start();
            this.sensorsAvailable.magnetometer = true;
            console.log('[SensorFusion] Magnetometer 활성화');
        } catch (e) {
            console.warn('[SensorFusion] Magnetometer 초기화 실패:', e);
        }
    }

    /**
     * DeviceOrientation 이벤트 핸들러
     */
    onDeviceOrientation(event) {
        if (event.alpha === null) return;

        const now = performance.now();
        this.deltaTime = this.lastTimestamp ? (now - this.lastTimestamp) / 1000 : 0.016;
        this.lastTimestamp = now;

        // 저주파 필터 적용
        const lp = this.lowPassFilter;
        lp.gyro.alpha = lp.alpha * lp.gyro.alpha + (1 - lp.alpha) * event.alpha;
        lp.gyro.beta = lp.alpha * lp.gyro.beta + (1 - lp.alpha) * event.beta;
        lp.gyro.gamma = lp.alpha * lp.gyro.gamma + (1 - lp.alpha) * event.gamma;

        this.gyroscope = {
            alpha: lp.gyro.alpha,
            beta: lp.gyro.beta,
            gamma: lp.gyro.gamma
        };

        // 초기 기준 설정
        if (!this.initialOrientation) {
            this.initialOrientation = { ...this.gyroscope };
            console.log('[SensorFusion] 초기 방향 설정:', this.initialOrientation);
        }

        // 회전 계산
        this.updateOrientation();
    }

    /**
     * DeviceMotion 이벤트 핸들러
     */
    onDeviceMotion(event) {
        const accel = event.accelerationIncludingGravity;
        if (!accel || accel.x === null) return;

        // 저주파 필터 적용
        const lp = this.lowPassFilter;
        lp.accel.x = lp.alpha * lp.accel.x + (1 - lp.alpha) * accel.x;
        lp.accel.y = lp.alpha * lp.accel.y + (1 - lp.alpha) * accel.y;
        lp.accel.z = lp.alpha * lp.accel.z + (1 - lp.alpha) * accel.z;

        this.accelerometer = {
            x: lp.accel.x,
            y: lp.accel.y,
            z: lp.accel.z
        };

        // 초기 가속도 (중력 방향) 설정
        if (!this.initialAccel) {
            this.initialAccel = { ...this.accelerometer };
            console.log('[SensorFusion] 초기 가속도 설정:', this.initialAccel);
            this.calibrated = true;
        }

        // 위치 업데이트 (가속도 적분)
        this.updatePositionFromAccel();
    }

    /**
     * 회전 업데이트 (상보 필터 + 칼만 필터)
     */
    updateOrientation() {
        if (!this.initialOrientation) return;

        // === 1. 자이로스코프 기반 회전 ===
        const alpha = THREE.MathUtils.degToRad(this.gyroscope.alpha);
        const beta = THREE.MathUtils.degToRad(this.gyroscope.beta);
        const gamma = THREE.MathUtils.degToRad(this.gyroscope.gamma);

        // Euler → Quaternion
        const euler = new THREE.Euler(beta, alpha, -gamma, 'YXZ');
        const gyroQuat = new THREE.Quaternion().setFromEuler(euler);

        // 화면 방향 보정
        const screenOrientation = window.orientation || 0;
        const screenQuat = new THREE.Quaternion().setFromAxisAngle(
            new THREE.Vector3(0, 0, 1),
            -THREE.MathUtils.degToRad(screenOrientation)
        );
        gyroQuat.multiply(screenQuat);

        // === 2. 가속도계 기반 틸트 보정 (상보 필터) ===
        if (this.sensorsAvailable.deviceMotion) {
            const accelQuat = this.getAccelOrientation();

            // 상보 필터: 자이로 (빠름) + 가속도 (드리프트 보정)
            gyroQuat.slerp(accelQuat, 1 - this.complementaryAlpha);
        }

        // === 3. 지자기계 기반 방향 보정 ===
        if (this.sensorsAvailable.magnetometer) {
            const magHeading = this.getMagneticHeading();
            // 북쪽 방향 보정 (TODO: 구현)
        }

        // === 4. 칼만 필터 적용 ===
        this.applyKalmanFilter(gyroQuat);

        // === 5. 초기 방향 보정 ===
        if (!this.kalman.orientation.initialInverse) {
            this.kalman.orientation.initialInverse = this.kalman.orientation.estimate.clone().invert();
        }

        // 최종 회전
        this.orientation.copy(this.kalman.orientation.estimate);
        this.orientation.premultiply(this.kalman.orientation.initialInverse);
    }

    /**
     * 가속도계에서 틸트 방향 계산
     */
    getAccelOrientation() {
        const ax = this.accelerometer.x;
        const ay = this.accelerometer.y;
        const az = this.accelerometer.z;

        // 중력 방향에서 pitch, roll 계산
        const pitch = Math.atan2(-ax, Math.sqrt(ay * ay + az * az));
        const roll = Math.atan2(ay, az);

        const euler = new THREE.Euler(pitch, 0, roll, 'YXZ');
        return new THREE.Quaternion().setFromEuler(euler);
    }

    /**
     * 지자기계에서 방위 계산
     */
    getMagneticHeading() {
        const mx = this.magnetometer.x;
        const my = this.magnetometer.y;

        // 자북 방향
        return Math.atan2(my, mx);
    }

    /**
     * 칼만 필터 적용 (회전)
     */
    applyKalmanFilter(measurement) {
        const k = this.kalman.orientation;

        // 예측 단계
        // (회전은 이전 상태 유지)

        // 칼만 이득 계산
        const kalmanGain = k.errorCovariance / (k.errorCovariance + k.measurementNoise);

        // 업데이트 단계
        k.estimate.slerp(measurement, kalmanGain);

        // 오차 공분산 업데이트
        k.errorCovariance = (1 - kalmanGain) * k.errorCovariance + k.processNoise;
    }

    /**
     * 가속도에서 위치 업데이트
     */
    updatePositionFromAccel() {
        if (!this.calibrated || this.deltaTime <= 0) return;

        // 중력 제거
        const gravity = 9.81;
        const ax = this.accelerometer.x - (this.initialAccel?.x || 0);
        const ay = this.accelerometer.y - (this.initialAccel?.y || 0);
        const az = this.accelerometer.z - (this.initialAccel?.z || 0);

        // 임계값 이하 무시 (노이즈)
        const threshold = 0.1;
        const accelMag = Math.sqrt(ax * ax + ay * ay + az * az);
        if (accelMag < threshold) {
            // 정지 상태 - 속도 감쇠
            this.velocity.multiplyScalar(0.95);
            return;
        }

        // 월드 좌표로 변환
        const accelWorld = new THREE.Vector3(ax, ay, az);
        accelWorld.applyQuaternion(this.orientation);

        // 속도 적분
        this.velocity.x += accelWorld.x * this.deltaTime;
        this.velocity.y += accelWorld.y * this.deltaTime;
        this.velocity.z += accelWorld.z * this.deltaTime;

        // 속도 감쇠 (드리프트 방지)
        this.velocity.multiplyScalar(0.98);

        // 위치 적분
        this.position.x += this.velocity.x * this.deltaTime;
        this.position.y += this.velocity.y * this.deltaTime;
        this.position.z += this.velocity.z * this.deltaTime;
    }

    /**
     * Visual Odometry 데이터 업데이트
     */
    updateFromVO(voPosition) {
        if (!voPosition) return;

        this.voEnabled = true;
        this.voPosition.copy(voPosition);

        // VO와 가속도계 퓨전
        const k = this.kalman.position;

        // 칼만 이득
        const kalmanGain = k.errorCovariance / (k.errorCovariance + k.measurementNoise);

        // VO 측정값으로 보정
        k.estimate.lerp(voPosition, kalmanGain);

        // 가속도계 기반 위치도 보정
        this.position.lerp(voPosition, 0.3);

        k.errorCovariance = (1 - kalmanGain) * k.errorCovariance + k.processNoise;
    }

    /**
     * 최종 회전 반환
     */
    getOrientation() {
        return this.orientation.clone();
    }

    /**
     * 최종 위치 반환
     */
    getPosition() {
        // VO가 있으면 VO 우선, 없으면 가속도계 기반
        if (this.voEnabled) {
            // VO + 가속도계 퓨전
            return this.kalman.position.estimate.clone();
        }
        return this.position.clone();
    }

    /**
     * 캘리브레이션 리셋
     */
    resetCalibration() {
        this.initialOrientation = null;
        this.initialAccel = null;
        this.calibrated = false;
        this.kalman.orientation.initialInverse = null;

        this.position.set(0, 0, 0);
        this.velocity.set(0, 0, 0);
        this.kalman.position.estimate.set(0, 0, 0);

        console.log('[SensorFusion] 캘리브레이션 리셋');
    }

    /**
     * iOS 권한 요청 (버튼 클릭 시 호출)
     */
    async requestPermissions() {
        let granted = false;

        if (typeof DeviceOrientationEvent.requestPermission === 'function') {
            try {
                const permission = await DeviceOrientationEvent.requestPermission();
                if (permission === 'granted') {
                    await this.initDeviceOrientation();
                    granted = true;
                }
            } catch (e) {
                console.error('[SensorFusion] 권한 요청 실패:', e);
            }
        }

        if (typeof DeviceMotionEvent.requestPermission === 'function') {
            try {
                const permission = await DeviceMotionEvent.requestPermission();
                if (permission === 'granted') {
                    await this.initDeviceMotion();
                    granted = true;
                }
            } catch (e) {
                console.error('[SensorFusion] 권한 요청 실패:', e);
            }
        }

        return granted;
    }

    /**
     * 디버그 정보 반환
     */
    getDebugInfo() {
        return {
            sensors: this.sensorsAvailable,
            calibrated: this.calibrated,
            gyro: this.gyroscope,
            accel: this.accelerometer,
            position: {
                x: this.position.x.toFixed(3),
                y: this.position.y.toFixed(3),
                z: this.position.z.toFixed(3)
            },
            velocity: {
                x: this.velocity.x.toFixed(3),
                y: this.velocity.y.toFixed(3),
                z: this.velocity.z.toFixed(3)
            }
        };
    }

    /**
     * 정리
     */
    destroy() {
        window.removeEventListener('deviceorientation', this.onDeviceOrientation);
        window.removeEventListener('devicemotion', this.onDeviceMotion);
        console.log('[SensorFusion] 종료');
    }
}

export default SensorFusion;
