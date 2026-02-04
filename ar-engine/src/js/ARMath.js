/**
 * ARMath.js
 *
 * C++ WebAssembly 수학 모듈과 Three.js를 연결하는 JavaScript 래퍼
 * 벡터/행렬 데이터를 효율적으로 C++과 JS 간에 교환
 */

let wasmModule = null;
let isInitialized = false;

/**
 * WebAssembly 모듈 초기화
 * @returns {Promise<void>}
 */
export async function initARMath() {
    if (isInitialized) {
        return;
    }

    try {
        // Wasm 모듈 로드 (webpack 또는 직접 로드)
        if (typeof createARMathModule === 'undefined') {
            // 동적 스크립트 로드
            await loadScript('/wasm/ar-math.js');
        }

        wasmModule = await createARMathModule();
        isInitialized = true;
        console.log('[ARMath] WebAssembly module initialized');
        console.log('[ARMath] ' + wasmModule.helloWorld());
    } catch (error) {
        console.error('[ARMath] Failed to initialize:', error);
        throw error;
    }
}

/**
 * 스크립트 동적 로드 헬퍼
 */
function loadScript(src) {
    return new Promise((resolve, reject) => {
        const script = document.createElement('script');
        script.src = src;
        script.onload = resolve;
        script.onerror = reject;
        document.head.appendChild(script);
    });
}

/**
 * 모듈 초기화 확인
 */
function ensureInitialized() {
    if (!isInitialized || !wasmModule) {
        throw new Error('[ARMath] Module not initialized. Call initARMath() first.');
    }
}

// ============================================================================
// Vec3 유틸리티
// ============================================================================

export const Vec3 = {
    /**
     * Vec3 생성
     * @param {number} x
     * @param {number} y
     * @param {number} z
     * @returns {object} C++ Vec3 객체
     */
    create(x = 0, y = 0, z = 0) {
        ensureInitialized();
        return wasmModule.vec3_create(x, y, z);
    },

    /**
     * 배열로 변환 [x, y, z]
     * @param {object} vec C++ Vec3
     * @returns {number[]}
     */
    toArray(vec) {
        ensureInitialized();
        return wasmModule.vec3_toArray(vec);
    },

    /**
     * 배열에서 생성
     * @param {number[]} arr [x, y, z]
     * @returns {object} C++ Vec3
     */
    fromArray(arr) {
        ensureInitialized();
        return wasmModule.vec3_fromArray(arr);
    },

    /**
     * 벡터 덧셈
     */
    add(a, b) {
        ensureInitialized();
        return wasmModule.vec3_add(a, b);
    },

    /**
     * 벡터 뺄셈
     */
    sub(a, b) {
        ensureInitialized();
        return wasmModule.vec3_sub(a, b);
    },

    /**
     * 스칼라 곱
     */
    scale(v, s) {
        ensureInitialized();
        return wasmModule.vec3_scale(v, s);
    },

    /**
     * 내적
     */
    dot(a, b) {
        ensureInitialized();
        return wasmModule.vec3_dot(a, b);
    },

    /**
     * 외적
     */
    cross(a, b) {
        ensureInitialized();
        return wasmModule.vec3_cross(a, b);
    },

    /**
     * 벡터 길이
     */
    length(v) {
        ensureInitialized();
        return wasmModule.vec3_length(v);
    },

    /**
     * 정규화
     */
    normalize(v) {
        ensureInitialized();
        return wasmModule.vec3_normalize(v);
    },

    /**
     * Three.js Vector3로 변환
     * @param {object} vec C++ Vec3
     * @param {THREE.Vector3} [out] 기존 객체에 덮어쓰기 (선택)
     * @returns {THREE.Vector3}
     */
    toThreeVector3(vec, out = null) {
        const arr = this.toArray(vec);
        if (out) {
            out.set(arr[0], arr[1], arr[2]);
            return out;
        }
        // THREE가 전역에 있다고 가정
        if (typeof THREE !== 'undefined') {
            return new THREE.Vector3(arr[0], arr[1], arr[2]);
        }
        return { x: arr[0], y: arr[1], z: arr[2] };
    },

    /**
     * Three.js Vector3에서 생성
     * @param {THREE.Vector3} threeVec
     * @returns {object} C++ Vec3
     */
    fromThreeVector3(threeVec) {
        return this.fromArray([threeVec.x, threeVec.y, threeVec.z]);
    }
};

// ============================================================================
// Mat4 유틸리티
// ============================================================================

export const Mat4 = {
    /**
     * 단위행렬 생성
     */
    identity() {
        ensureInitialized();
        return wasmModule.mat4_identity();
    },

    /**
     * 이동 행렬
     */
    translation(x, y, z) {
        ensureInitialized();
        return wasmModule.mat4_translation(x, y, z);
    },

    /**
     * 스케일 행렬
     */
    scale(x, y, z) {
        ensureInitialized();
        return wasmModule.mat4_scale(x, y, z);
    },

    /**
     * X축 회전 (라디안)
     */
    rotationX(radians) {
        ensureInitialized();
        return wasmModule.mat4_rotationX(radians);
    },

    /**
     * Y축 회전 (라디안)
     */
    rotationY(radians) {
        ensureInitialized();
        return wasmModule.mat4_rotationY(radians);
    },

    /**
     * Z축 회전 (라디안)
     */
    rotationZ(radians) {
        ensureInitialized();
        return wasmModule.mat4_rotationZ(radians);
    },

    /**
     * 원근 투영 행렬
     */
    perspective(fovY, aspect, near, far) {
        ensureInitialized();
        return wasmModule.mat4_perspective(fovY, aspect, near, far);
    },

    /**
     * LookAt 뷰 행렬
     */
    lookAt(eye, target, up) {
        ensureInitialized();
        return wasmModule.mat4_lookAt(eye, target, up);
    },

    /**
     * 행렬 곱셈
     */
    multiply(a, b) {
        ensureInitialized();
        return wasmModule.mat4_multiply(a, b);
    },

    /**
     * 점 변환
     */
    transformPoint(mat, point) {
        ensureInitialized();
        return wasmModule.mat4_transformPoint(mat, point);
    },

    /**
     * 방향 벡터 변환
     */
    transformDirection(mat, dir) {
        ensureInitialized();
        return wasmModule.mat4_transformDirection(mat, dir);
    },

    /**
     * 전치행렬
     */
    transpose(mat) {
        ensureInitialized();
        return wasmModule.mat4_transpose(mat);
    },

    /**
     * Float32Array로 변환 (WebGL용, Column-Major)
     * @param {object} mat C++ Mat4
     * @returns {Float32Array}
     */
    toFloat32Array(mat) {
        ensureInitialized();
        const arr = wasmModule.mat4_toArray(mat);
        return new Float32Array(arr);
    },

    /**
     * 배열로 변환
     */
    toArray(mat) {
        ensureInitialized();
        return wasmModule.mat4_toArray(mat);
    },

    /**
     * 배열에서 생성
     */
    fromArray(arr) {
        ensureInitialized();
        return wasmModule.mat4_fromArray(arr);
    },

    /**
     * Three.js Matrix4로 변환
     * @param {object} mat C++ Mat4
     * @param {THREE.Matrix4} [out] 기존 객체에 덮어쓰기 (선택)
     * @returns {THREE.Matrix4}
     */
    toThreeMatrix4(mat, out = null) {
        const arr = this.toArray(mat);
        if (out) {
            out.fromArray(arr);
            return out;
        }
        if (typeof THREE !== 'undefined') {
            const m = new THREE.Matrix4();
            m.fromArray(arr);
            return m;
        }
        return arr;
    },

    /**
     * Three.js Matrix4에서 생성
     * @param {THREE.Matrix4} threeMat
     * @returns {object} C++ Mat4
     */
    fromThreeMatrix4(threeMat) {
        const arr = threeMat.toArray();
        return this.fromArray(arr);
    }
};

// ============================================================================
// Quaternion 유틸리티
// ============================================================================

export const Quat = {
    /**
     * 단위 쿼터니언
     */
    identity() {
        ensureInitialized();
        return wasmModule.quat_identity();
    },

    /**
     * 축-각도로 생성
     * @param {object} axis C++ Vec3
     * @param {number} angle 라디안
     */
    fromAxisAngle(axis, angle) {
        ensureInitialized();
        return wasmModule.quat_fromAxisAngle(axis, angle);
    },

    /**
     * 오일러각으로 생성
     * @param {number} pitch X축 회전 (라디안)
     * @param {number} yaw Y축 회전 (라디안)
     * @param {number} roll Z축 회전 (라디안)
     */
    fromEuler(pitch, yaw, roll) {
        ensureInitialized();
        return wasmModule.quat_fromEuler(pitch, yaw, roll);
    },

    /**
     * 회전 행렬로 변환
     */
    toMatrix(quat) {
        ensureInitialized();
        return wasmModule.quat_toMatrix(quat);
    },

    /**
     * 정규화
     */
    normalize(quat) {
        ensureInitialized();
        return wasmModule.quat_normalize(quat);
    },

    /**
     * 배열로 변환 [x, y, z, w]
     */
    toArray(quat) {
        ensureInitialized();
        return wasmModule.quat_toArray(quat);
    },

    /**
     * Three.js Quaternion으로 변환
     */
    toThreeQuaternion(quat, out = null) {
        const arr = this.toArray(quat);
        if (out) {
            out.set(arr[0], arr[1], arr[2], arr[3]);
            return out;
        }
        if (typeof THREE !== 'undefined') {
            return new THREE.Quaternion(arr[0], arr[1], arr[2], arr[3]);
        }
        return { x: arr[0], y: arr[1], z: arr[2], w: arr[3] };
    }
};

// ============================================================================
// 테스트/유틸리티 함수
// ============================================================================

/**
 * C++ 벡터 연산 테스트
 * @param {number[]} v1 [x, y, z]
 * @param {number[]} v2 [x, y, z]
 * @returns {object} 연산 결과
 */
export function testVectorOperations(v1, v2) {
    ensureInitialized();
    return wasmModule.testVectorOperations(v1, v2);
}

/**
 * C++ 행렬 연산 테스트
 * @param {number} tx 이동 X
 * @param {number} ty 이동 Y
 * @param {number} tz 이동 Z
 * @param {number} angle 회전각 (라디안)
 * @returns {object} 변환 결과
 */
export function testMatrixOperations(tx, ty, tz, angle) {
    ensureInitialized();
    return wasmModule.testMatrixOperations(tx, ty, tz, angle);
}

/**
 * MVP 행렬 생성 (Three.js 카메라 대체 가능)
 */
export function createMVPMatrix(eye, target, fovY, aspect, near, far) {
    ensureInitialized();
    return wasmModule.createMVPMatrix(
        eye[0], eye[1], eye[2],
        target[0], target[1], target[2],
        fovY, aspect, near, far
    );
}

/**
 * Hello World 테스트
 */
export function helloWorld() {
    ensureInitialized();
    return wasmModule.helloWorld();
}

/**
 * 모듈 초기화 상태 확인
 */
export function isARMathReady() {
    return isInitialized && wasmModule !== null;
}

/**
 * Raw Wasm 모듈 접근 (고급 사용)
 */
export function getWasmModule() {
    ensureInitialized();
    return wasmModule;
}

// 기본 내보내기
export default {
    init: initARMath,
    isReady: isARMathReady,
    Vec3,
    Mat4,
    Quat,
    helloWorld,
    testVectorOperations,
    testMatrixOperations,
    createMVPMatrix,
    getWasmModule
};
