/**
 * CameraPoseManager.js
 *
 * Wasm에서 계산된 4x4 카메라 행렬(Pose Matrix)을
 * Three.js PerspectiveCamera에 정확히 매칭하는 모듈
 *
 * 핵심 개념:
 * - Wasm(OpenCV)의 카메라 좌표계: Y-down, Z-forward (컴퓨터 비전)
 * - Three.js/WebGL 좌표계: Y-up, Z-backward (OpenGL)
 * - 이 차이를 보정하여 폰 움직임과 가상 카메라가 1:1 일치하도록 함
 */

import * as THREE from 'three';

/**
 * 좌표계 변환 상수
 * OpenCV → OpenGL 변환 행렬
 */
const CV_TO_GL = new THREE.Matrix4().set(
    1,  0,  0, 0,
    0, -1,  0, 0,  // Y축 반전
    0,  0, -1, 0,  // Z축 반전
    0,  0,  0, 1
);

/**
 * CameraPoseManager 클래스
 */
export class CameraPoseManager {
    constructor(camera) {
        if (!camera || !(camera instanceof THREE.PerspectiveCamera)) {
            throw new Error('THREE.PerspectiveCamera 인스턴스가 필요합니다');
        }

        this.camera = camera;

        // Three.js 카메라 설정: matrixAutoUpdate를 끄고 직접 행렬 제어
        this.camera.matrixAutoUpdate = false;

        // 변환 행렬들
        this.viewMatrix = new THREE.Matrix4();
        this.poseMatrix = new THREE.Matrix4();  // View의 역행렬 = 카메라 포즈

        // 좌표계 변환
        this.cvToGl = CV_TO_GL.clone();

        // 보정 행렬 (캘리브레이션용)
        this.calibrationMatrix = new THREE.Matrix4();

        // 초기 포즈 저장 (상대 좌표 계산용)
        this.initialPose = null;
        this.useRelativePose = true;  // 상대 좌표 사용 여부

        // 스무딩
        this.smoothingEnabled = true;
        this.smoothingFactor = 0.3;  // 0 = 스무딩 없음, 1 = 완전 고정
        this.lastPose = new THREE.Matrix4();
        this.hasLastPose = false;

        // 상태
        this.isTracking = false;
        this.frameCount = 0;

        console.log('[CameraPose] 초기화 완료');
    }

    /**
     * Wasm에서 받은 4x4 View Matrix를 적용
     *
     * @param {Float32Array|number[]} viewMatrixData 16개 요소의 column-major 행렬
     * @param {boolean} isColumnMajor true면 column-major (WebGL 기본), false면 row-major
     */
    applyViewMatrix(viewMatrixData, isColumnMajor = true) {
        if (!viewMatrixData || viewMatrixData.length !== 16) {
            console.warn('[CameraPose] 유효하지 않은 행렬 데이터');
            return false;
        }

        // Float32Array → THREE.Matrix4
        if (isColumnMajor) {
            this.viewMatrix.fromArray(viewMatrixData);
        } else {
            // Row-major인 경우 전치 필요
            this.viewMatrix.fromArray(viewMatrixData);
            this.viewMatrix.transpose();
        }

        // View Matrix → Pose Matrix (카메라 월드 위치/방향)
        // Pose = View^(-1)
        this.poseMatrix.copy(this.viewMatrix).invert();

        // OpenCV → OpenGL 좌표계 변환
        this.poseMatrix.premultiply(this.cvToGl);

        // 초기 포즈 저장 (첫 프레임)
        if (!this.initialPose && this.useRelativePose) {
            this.initialPose = this.poseMatrix.clone();
            console.log('[CameraPose] 초기 포즈 저장됨');
        }

        // 상대 좌표 계산 (초기 위치를 원점으로)
        if (this.initialPose && this.useRelativePose) {
            const initialInverse = this.initialPose.clone().invert();
            this.poseMatrix.premultiply(initialInverse);
        }

        // 캘리브레이션 행렬 적용
        this.poseMatrix.multiply(this.calibrationMatrix);

        // 스무딩 적용
        if (this.smoothingEnabled && this.hasLastPose) {
            this.applySmoothingToMatrix(this.poseMatrix, this.lastPose, this.smoothingFactor);
        }

        // Three.js 카메라에 적용
        this.camera.matrix.copy(this.poseMatrix);
        this.camera.matrixWorldNeedsUpdate = true;

        // 상태 업데이트
        this.lastPose.copy(this.poseMatrix);
        this.hasLastPose = true;
        this.isTracking = true;
        this.frameCount++;

        return true;
    }

    /**
     * 포즈 데이터(쿼터니언 + 이동)를 직접 적용
     *
     * @param {Object} pose { quaternion: {x,y,z,w}, position: {x,y,z} }
     */
    applyPose(pose) {
        if (!pose || !pose.quaternion || !pose.position) {
            return false;
        }

        const { quaternion, position } = pose;

        // Quaternion과 Position으로 행렬 구성
        const quat = new THREE.Quaternion(
            quaternion.x,
            quaternion.y,
            quaternion.z,
            quaternion.w
        );
        const pos = new THREE.Vector3(
            position.x,
            position.y,
            position.z
        );

        // 좌표계 변환 (Y, Z 반전)
        pos.y = -pos.y;
        pos.z = -pos.z;

        // 쿼터니언도 좌표계 변환
        // OpenCV → OpenGL: Y, Z 축 반전
        quat.x = -quat.x;
        quat.w = -quat.w;

        this.poseMatrix.compose(pos, quat, new THREE.Vector3(1, 1, 1));

        // 초기 포즈 처리
        if (!this.initialPose && this.useRelativePose) {
            this.initialPose = this.poseMatrix.clone();
        }

        if (this.initialPose && this.useRelativePose) {
            const initialInverse = this.initialPose.clone().invert();
            this.poseMatrix.premultiply(initialInverse);
        }

        // 스무딩
        if (this.smoothingEnabled && this.hasLastPose) {
            this.applySmoothingToMatrix(this.poseMatrix, this.lastPose, this.smoothingFactor);
        }

        // 카메라 적용
        this.camera.matrix.copy(this.poseMatrix);
        this.camera.matrixWorldNeedsUpdate = true;

        this.lastPose.copy(this.poseMatrix);
        this.hasLastPose = true;
        this.isTracking = true;
        this.frameCount++;

        return true;
    }

    /**
     * DeviceOrientation 센서 데이터로 카메라 회전 (폴백용)
     *
     * @param {number} alpha Z축 회전 (0-360)
     * @param {number} beta X축 회전 (-180 ~ 180)
     * @param {number} gamma Y축 회전 (-90 ~ 90)
     * @param {number} screenOrientation 화면 방향 (0, 90, -90, 180)
     */
    applyDeviceOrientation(alpha, beta, gamma, screenOrientation = 0) {
        // 도 → 라디안
        const alphaRad = THREE.MathUtils.degToRad(alpha);
        const betaRad = THREE.MathUtils.degToRad(beta);
        const gammaRad = THREE.MathUtils.degToRad(gamma);
        const screenRad = THREE.MathUtils.degToRad(screenOrientation);

        // 디바이스 방향 → Three.js 카메라 방향 변환
        // ZXY 오일러 순서 사용 (디바이스 센서 기준)
        const euler = new THREE.Euler();
        euler.set(betaRad, alphaRad, -gammaRad, 'YXZ');

        const quaternion = new THREE.Quaternion();
        quaternion.setFromEuler(euler);

        // 화면 방향 보정
        const screenQuat = new THREE.Quaternion();
        screenQuat.setFromAxisAngle(new THREE.Vector3(0, 0, 1), -screenRad);
        quaternion.multiply(screenQuat);

        // 디바이스가 세로 방향일 때 -90도 X축 회전 보정
        const worldQuat = new THREE.Quaternion();
        worldQuat.setFromAxisAngle(new THREE.Vector3(1, 0, 0), -Math.PI / 2);
        quaternion.premultiply(worldQuat);

        // 카메라에 적용 (회전만, 위치는 유지)
        this.camera.quaternion.copy(quaternion);

        // matrixAutoUpdate가 꺼져 있으면 수동으로 행렬 업데이트
        if (!this.camera.matrixAutoUpdate) {
            this.camera.updateMatrix();
        }
    }

    /**
     * 행렬 스무딩 (떨림 방지)
     */
    applySmoothingToMatrix(target, previous, factor) {
        // 위치 스무딩
        const pos = new THREE.Vector3();
        const prevPos = new THREE.Vector3();
        pos.setFromMatrixPosition(target);
        prevPos.setFromMatrixPosition(previous);
        pos.lerp(prevPos, factor);

        // 회전 스무딩 (쿼터니언 SLERP)
        const quat = new THREE.Quaternion();
        const prevQuat = new THREE.Quaternion();
        quat.setFromRotationMatrix(target);
        prevQuat.setFromRotationMatrix(previous);
        quat.slerp(prevQuat, factor);

        // 스무딩된 값으로 행렬 재구성
        const scale = new THREE.Vector3(1, 1, 1);
        target.compose(pos, quat, scale);
    }

    /**
     * 카메라 파라미터를 Three.js에 동기화
     *
     * @param {number} fx 초점 거리 X (픽셀)
     * @param {number} fy 초점 거리 Y (픽셀)
     * @param {number} cx 주점 X (픽셀)
     * @param {number} cy 주점 Y (픽셀)
     * @param {number} width 이미지 너비
     * @param {number} height 이미지 높이
     */
    syncCameraIntrinsics(fx, fy, cx, cy, width, height) {
        // FOV 계산: fov = 2 * atan(height / (2 * fy))
        const fovY = 2 * Math.atan(height / (2 * fy)) * (180 / Math.PI);

        this.camera.fov = fovY;
        this.camera.aspect = width / height;
        this.camera.updateProjectionMatrix();

        console.log('[CameraPose] 카메라 파라미터 동기화:', {
            fov: fovY.toFixed(1),
            aspect: (width / height).toFixed(2)
        });
    }

    /**
     * 캘리브레이션 설정
     */
    setCalibration(rotationOffset, positionOffset) {
        const rot = new THREE.Quaternion();
        if (rotationOffset) {
            rot.setFromEuler(new THREE.Euler(
                rotationOffset.x || 0,
                rotationOffset.y || 0,
                rotationOffset.z || 0
            ));
        }

        const pos = new THREE.Vector3(
            positionOffset?.x || 0,
            positionOffset?.y || 0,
            positionOffset?.z || 0
        );

        this.calibrationMatrix.compose(pos, rot, new THREE.Vector3(1, 1, 1));
    }

    /**
     * 스무딩 설정
     */
    setSmoothing(enabled, factor = 0.3) {
        this.smoothingEnabled = enabled;
        this.smoothingFactor = Math.max(0, Math.min(1, factor));
    }

    /**
     * 상대 좌표 사용 설정
     */
    setRelativePose(enabled) {
        this.useRelativePose = enabled;
        if (!enabled) {
            this.initialPose = null;
        }
    }

    /**
     * 리셋 (초기 포즈 재설정)
     */
    reset() {
        this.initialPose = null;
        this.hasLastPose = false;
        this.isTracking = false;
        this.frameCount = 0;

        // 카메라 원점으로 리셋
        this.camera.matrix.identity();
        this.camera.matrixWorldNeedsUpdate = true;

        console.log('[CameraPose] 리셋됨');
    }

    /**
     * 현재 카메라 위치/방향 반환
     */
    getCameraPosition() {
        const pos = new THREE.Vector3();
        pos.setFromMatrixPosition(this.camera.matrix);
        return pos;
    }

    getCameraQuaternion() {
        const quat = new THREE.Quaternion();
        quat.setFromRotationMatrix(this.camera.matrix);
        return quat;
    }

    getCameraDirection() {
        const dir = new THREE.Vector3(0, 0, -1);
        dir.applyQuaternion(this.getCameraQuaternion());
        return dir;
    }

    /**
     * 디버그 정보
     */
    getDebugInfo() {
        const pos = this.getCameraPosition();
        const dir = this.getCameraDirection();

        return {
            position: { x: pos.x.toFixed(3), y: pos.y.toFixed(3), z: pos.z.toFixed(3) },
            direction: { x: dir.x.toFixed(3), y: dir.y.toFixed(3), z: dir.z.toFixed(3) },
            isTracking: this.isTracking,
            frameCount: this.frameCount
        };
    }
}

/**
 * 바닥 기준 객체 배치를 위한 유틸리티
 */
export class ARObjectPlacer {
    constructor(scene, camera) {
        this.scene = scene;
        this.camera = camera;
    }

    /**
     * 월드 원점 (0, 0, 0)에 객체 배치
     * AR에서 초기 위치가 원점이 됨
     */
    placeAtOrigin(object) {
        object.position.set(0, 0, 0);
        this.scene.add(object);
        return object;
    }

    /**
     * 바닥 (y=0)에 객체 배치
     */
    placeOnFloor(object, x = 0, z = 0) {
        // 객체의 바운딩 박스를 고려하여 바닥에 맞춤
        const box = new THREE.Box3().setFromObject(object);
        const height = box.max.y - box.min.y;

        object.position.set(x, height / 2, z);
        this.scene.add(object);
        return object;
    }

    /**
     * 카메라 앞 특정 거리에 배치
     */
    placeInFrontOfCamera(object, distance = 1) {
        const dir = new THREE.Vector3(0, 0, -1);
        dir.applyQuaternion(this.camera.quaternion);

        const pos = this.camera.position.clone();
        pos.add(dir.multiplyScalar(distance));

        object.position.copy(pos);
        this.scene.add(object);
        return object;
    }

    /**
     * 빨간 큐브 생성 헬퍼
     */
    static createRedCube(size = 0.1) {
        const geometry = new THREE.BoxGeometry(size, size, size);
        const material = new THREE.MeshPhongMaterial({
            color: 0xff0000,
            shininess: 100
        });
        return new THREE.Mesh(geometry, material);
    }

    /**
     * 바닥 그리드 생성 헬퍼
     */
    static createFloorGrid(size = 10, divisions = 10) {
        return new THREE.GridHelper(size, divisions, 0x444444, 0x222222);
    }

    /**
     * 축 헬퍼 생성 (X=빨강, Y=초록, Z=파랑)
     */
    static createAxesHelper(size = 1) {
        return new THREE.AxesHelper(size);
    }
}

export default CameraPoseManager;
