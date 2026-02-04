/**
 * math_bindings.cpp
 *
 * C++ 수학 타입(Vec3, Mat4, Quaternion)을 JavaScript와 연결하는 Emscripten 바인딩
 * 벡터/행렬 데이터를 효율적으로 주고받기 위한 Hello World급 예제
 */

#include <emscripten/bind.h>
#include <emscripten/val.h>
#include "math_types.h"
#include <vector>
#include <string>

using namespace emscripten;
using namespace ar;

// ============================================================================
// JavaScript 배열 <-> C++ 변환 헬퍼
// ============================================================================

// JS Float32Array/Array -> std::vector<float>
std::vector<float> jsArrayToVector(const val& jsArray) {
    const size_t length = jsArray["length"].as<size_t>();
    std::vector<float> result(length);
    for (size_t i = 0; i < length; ++i) {
        result[i] = jsArray[i].as<float>();
    }
    return result;
}

// std::vector<float> -> JS Array
val vectorToJsArray(const std::vector<float>& vec) {
    val result = val::array();
    for (size_t i = 0; i < vec.size(); ++i) {
        result.call<void>("push", vec[i]);
    }
    return result;
}

// ============================================================================
// Vec3 래퍼 함수들
// ============================================================================

Vec3 vec3_create(float x, float y, float z) {
    return Vec3(x, y, z);
}

val vec3_toArray(const Vec3& v) {
    val result = val::array();
    result.call<void>("push", v.x);
    result.call<void>("push", v.y);
    result.call<void>("push", v.z);
    return result;
}

Vec3 vec3_fromArray(const val& arr) {
    return Vec3(
        arr[0].as<float>(),
        arr[1].as<float>(),
        arr[2].as<float>()
    );
}

Vec3 vec3_add(const Vec3& a, const Vec3& b) { return a + b; }
Vec3 vec3_sub(const Vec3& a, const Vec3& b) { return a - b; }
Vec3 vec3_scale(const Vec3& v, float s) { return v * s; }
float vec3_dot(const Vec3& a, const Vec3& b) { return a.dot(b); }
Vec3 vec3_cross(const Vec3& a, const Vec3& b) { return a.cross(b); }
float vec3_length(const Vec3& v) { return v.length(); }
Vec3 vec3_normalize(const Vec3& v) { return v.normalized(); }

// ============================================================================
// Mat4 래퍼 함수들
// ============================================================================

Mat4 mat4_identity() {
    return Mat4::identity();
}

Mat4 mat4_translation(float x, float y, float z) {
    return Mat4::translation(x, y, z);
}

Mat4 mat4_scale(float x, float y, float z) {
    return Mat4::scale(x, y, z);
}

Mat4 mat4_rotationX(float radians) {
    return Mat4::rotationX(radians);
}

Mat4 mat4_rotationY(float radians) {
    return Mat4::rotationY(radians);
}

Mat4 mat4_rotationZ(float radians) {
    return Mat4::rotationZ(radians);
}

Mat4 mat4_perspective(float fovY, float aspect, float near, float far) {
    return Mat4::perspective(fovY, aspect, near, far);
}

Mat4 mat4_lookAt(const Vec3& eye, const Vec3& target, const Vec3& up) {
    return Mat4::lookAt(eye, target, up);
}

Mat4 mat4_multiply(const Mat4& a, const Mat4& b) {
    return a * b;
}

Vec3 mat4_transformPoint(const Mat4& m, const Vec3& p) {
    return m.transformPoint(p);
}

Vec3 mat4_transformDirection(const Mat4& m, const Vec3& d) {
    return m.transformDirection(d);
}

Mat4 mat4_transpose(const Mat4& m) {
    return m.transposed();
}

// Mat4 -> JS Float32Array (16개 요소)
val mat4_toArray(const Mat4& m) {
    val result = val::array();
    for (int i = 0; i < 16; ++i) {
        result.call<void>("push", m.data[i]);
    }
    return result;
}

// JS Array -> Mat4
Mat4 mat4_fromArray(const val& arr) {
    Mat4 m;
    for (int i = 0; i < 16; ++i) {
        m.data[i] = arr[i].as<float>();
    }
    return m;
}

// ============================================================================
// Quaternion 래퍼 함수들
// ============================================================================

Quaternion quat_identity() {
    return Quaternion();
}

Quaternion quat_fromAxisAngle(const Vec3& axis, float angle) {
    return Quaternion::fromAxisAngle(axis, angle);
}

Quaternion quat_fromEuler(float pitch, float yaw, float roll) {
    return Quaternion::fromEuler(pitch, yaw, roll);
}

Mat4 quat_toMatrix(const Quaternion& q) {
    return q.toMatrix();
}

Quaternion quat_normalize(const Quaternion& q) {
    return q.normalized();
}

val quat_toArray(const Quaternion& q) {
    val result = val::array();
    result.call<void>("push", q.x);
    result.call<void>("push", q.y);
    result.call<void>("push", q.z);
    result.call<void>("push", q.w);
    return result;
}

// ============================================================================
// 테스트용 Hello World 함수들
// ============================================================================

std::string helloWorld() {
    return "Hello from C++ WebAssembly!";
}

// 벡터 연산 테스트: 두 벡터를 받아 내적과 외적 계산
val testVectorOperations(const val& v1Array, const val& v2Array) {
    Vec3 v1 = vec3_fromArray(v1Array);
    Vec3 v2 = vec3_fromArray(v2Array);

    val result = val::object();
    result.set("dot", v1.dot(v2));
    result.set("cross", vec3_toArray(v1.cross(v2)));
    result.set("v1Length", v1.length());
    result.set("v2Length", v2.length());
    result.set("sum", vec3_toArray(v1 + v2));
    return result;
}

// 행렬 연산 테스트: 변환 행렬 생성 및 점 변환
val testMatrixOperations(float tx, float ty, float tz, float angle) {
    // Model 행렬 생성: 이동 * 회전
    Mat4 translation = Mat4::translation(tx, ty, tz);
    Mat4 rotation = Mat4::rotationY(angle);
    Mat4 model = translation * rotation;

    // 원점을 변환
    Vec3 origin(0, 0, 0);
    Vec3 transformed = model.transformPoint(origin);

    val result = val::object();
    result.set("modelMatrix", mat4_toArray(model));
    result.set("transformedPoint", vec3_toArray(transformed));
    return result;
}

// MVP 행렬 계산 테스트 (Three.js와 연동 가능)
val createMVPMatrix(
    float eyeX, float eyeY, float eyeZ,
    float targetX, float targetY, float targetZ,
    float fovY, float aspect, float near, float far
) {
    Vec3 eye(eyeX, eyeY, eyeZ);
    Vec3 target(targetX, targetY, targetZ);
    Vec3 up(0, 1, 0);

    Mat4 view = Mat4::lookAt(eye, target, up);
    Mat4 projection = Mat4::perspective(fovY, aspect, near, far);
    Mat4 vp = projection * view;

    val result = val::object();
    result.set("viewMatrix", mat4_toArray(view));
    result.set("projectionMatrix", mat4_toArray(projection));
    result.set("viewProjectionMatrix", mat4_toArray(vp));
    return result;
}

// ============================================================================
// Emscripten 바인딩 정의
// ============================================================================

EMSCRIPTEN_BINDINGS(ar_math_module) {
    // Vec3 클래스 바인딩
    class_<Vec3>("Vec3")
        .constructor<>()
        .constructor<float, float, float>()
        .property("x", &Vec3::x)
        .property("y", &Vec3::y)
        .property("z", &Vec3::z);

    // Mat4 클래스 바인딩
    class_<Mat4>("Mat4")
        .constructor<>();

    // Quaternion 클래스 바인딩
    class_<Quaternion>("Quaternion")
        .constructor<>()
        .constructor<float, float, float, float>()
        .property("x", &Quaternion::x)
        .property("y", &Quaternion::y)
        .property("z", &Quaternion::z)
        .property("w", &Quaternion::w);

    // Vec3 함수들
    function("vec3_create", &vec3_create);
    function("vec3_toArray", &vec3_toArray);
    function("vec3_fromArray", &vec3_fromArray);
    function("vec3_add", &vec3_add);
    function("vec3_sub", &vec3_sub);
    function("vec3_scale", &vec3_scale);
    function("vec3_dot", &vec3_dot);
    function("vec3_cross", &vec3_cross);
    function("vec3_length", &vec3_length);
    function("vec3_normalize", &vec3_normalize);

    // Mat4 함수들
    function("mat4_identity", &mat4_identity);
    function("mat4_translation", &mat4_translation);
    function("mat4_scale", &mat4_scale);
    function("mat4_rotationX", &mat4_rotationX);
    function("mat4_rotationY", &mat4_rotationY);
    function("mat4_rotationZ", &mat4_rotationZ);
    function("mat4_perspective", &mat4_perspective);
    function("mat4_lookAt", &mat4_lookAt);
    function("mat4_multiply", &mat4_multiply);
    function("mat4_transformPoint", &mat4_transformPoint);
    function("mat4_transformDirection", &mat4_transformDirection);
    function("mat4_transpose", &mat4_transpose);
    function("mat4_toArray", &mat4_toArray);
    function("mat4_fromArray", &mat4_fromArray);

    // Quaternion 함수들
    function("quat_identity", &quat_identity);
    function("quat_fromAxisAngle", &quat_fromAxisAngle);
    function("quat_fromEuler", &quat_fromEuler);
    function("quat_toMatrix", &quat_toMatrix);
    function("quat_normalize", &quat_normalize);
    function("quat_toArray", &quat_toArray);

    // 테스트/유틸 함수들
    function("helloWorld", &helloWorld);
    function("testVectorOperations", &testVectorOperations);
    function("testMatrixOperations", &testMatrixOperations);
    function("createMVPMatrix", &createMVPMatrix);
}
