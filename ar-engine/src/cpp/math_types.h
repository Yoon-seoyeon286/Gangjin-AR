#ifndef MATH_TYPES_H
#define MATH_TYPES_H

#include <array>
#include <cmath>
#include <cstring>

namespace ar {

// ============================================================================
// Vec3: 3D 벡터
// ============================================================================
struct Vec3 {
    float x, y, z;

    Vec3() : x(0), y(0), z(0) {}
    Vec3(float x, float y, float z) : x(x), y(y), z(z) {}

    // 벡터 연산
    Vec3 operator+(const Vec3& v) const { return Vec3(x + v.x, y + v.y, z + v.z); }
    Vec3 operator-(const Vec3& v) const { return Vec3(x - v.x, y - v.y, z - v.z); }
    Vec3 operator*(float s) const { return Vec3(x * s, y * s, z * s); }

    float dot(const Vec3& v) const { return x * v.x + y * v.y + z * v.z; }

    Vec3 cross(const Vec3& v) const {
        return Vec3(
            y * v.z - z * v.y,
            z * v.x - x * v.z,
            x * v.y - y * v.x
        );
    }

    float length() const { return std::sqrt(x * x + y * y + z * z); }

    Vec3 normalized() const {
        float len = length();
        if (len > 0.0001f) {
            return Vec3(x / len, y / len, z / len);
        }
        return Vec3(0, 0, 0);
    }

    // 배열로 변환 (JS 전달용)
    std::array<float, 3> toArray() const {
        return {x, y, z};
    }

    // 배열에서 생성
    static Vec3 fromArray(const float* data) {
        return Vec3(data[0], data[1], data[2]);
    }
};

// ============================================================================
// Vec4: 4D 벡터 (동차좌표용)
// ============================================================================
struct Vec4 {
    float x, y, z, w;

    Vec4() : x(0), y(0), z(0), w(1) {}
    Vec4(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}
    Vec4(const Vec3& v, float w = 1.0f) : x(v.x), y(v.y), z(v.z), w(w) {}

    Vec3 toVec3() const { return Vec3(x, y, z); }
    Vec3 perspectiveDivide() const {
        if (std::abs(w) > 0.0001f) {
            return Vec3(x / w, y / w, z / w);
        }
        return Vec3(x, y, z);
    }

    std::array<float, 4> toArray() const {
        return {x, y, z, w};
    }
};

// ============================================================================
// Mat4: 4x4 행렬 (Column-Major, WebGL/OpenGL 호환)
// ============================================================================
struct Mat4 {
    // Column-major order: m[col][row] 또는 data[col * 4 + row]
    float data[16];

    Mat4() {
        std::memset(data, 0, sizeof(data));
        data[0] = data[5] = data[10] = data[15] = 1.0f; // 단위행렬
    }

    // 원소 접근 (row, col)
    float& at(int row, int col) { return data[col * 4 + row]; }
    float at(int row, int col) const { return data[col * 4 + row]; }

    // 단위행렬 생성
    static Mat4 identity() {
        return Mat4();
    }

    // 이동 행렬
    static Mat4 translation(float x, float y, float z) {
        Mat4 m;
        m.at(0, 3) = x;
        m.at(1, 3) = y;
        m.at(2, 3) = z;
        return m;
    }

    static Mat4 translation(const Vec3& v) {
        return translation(v.x, v.y, v.z);
    }

    // 스케일 행렬
    static Mat4 scale(float x, float y, float z) {
        Mat4 m;
        m.at(0, 0) = x;
        m.at(1, 1) = y;
        m.at(2, 2) = z;
        return m;
    }

    // X축 회전
    static Mat4 rotationX(float radians) {
        Mat4 m;
        float c = std::cos(radians);
        float s = std::sin(radians);
        m.at(1, 1) = c;  m.at(1, 2) = -s;
        m.at(2, 1) = s;  m.at(2, 2) = c;
        return m;
    }

    // Y축 회전
    static Mat4 rotationY(float radians) {
        Mat4 m;
        float c = std::cos(radians);
        float s = std::sin(radians);
        m.at(0, 0) = c;  m.at(0, 2) = s;
        m.at(2, 0) = -s; m.at(2, 2) = c;
        return m;
    }

    // Z축 회전
    static Mat4 rotationZ(float radians) {
        Mat4 m;
        float c = std::cos(radians);
        float s = std::sin(radians);
        m.at(0, 0) = c;  m.at(0, 1) = -s;
        m.at(1, 0) = s;  m.at(1, 1) = c;
        return m;
    }

    // Perspective 투영 행렬
    static Mat4 perspective(float fovY, float aspect, float near, float far) {
        Mat4 m;
        std::memset(m.data, 0, sizeof(m.data));

        float tanHalfFov = std::tan(fovY / 2.0f);
        m.at(0, 0) = 1.0f / (aspect * tanHalfFov);
        m.at(1, 1) = 1.0f / tanHalfFov;
        m.at(2, 2) = -(far + near) / (far - near);
        m.at(2, 3) = -(2.0f * far * near) / (far - near);
        m.at(3, 2) = -1.0f;
        return m;
    }

    // LookAt 뷰 행렬
    static Mat4 lookAt(const Vec3& eye, const Vec3& target, const Vec3& up) {
        Vec3 f = (target - eye).normalized();
        Vec3 r = f.cross(up).normalized();
        Vec3 u = r.cross(f);

        Mat4 m;
        m.at(0, 0) = r.x;  m.at(0, 1) = r.y;  m.at(0, 2) = r.z;  m.at(0, 3) = -r.dot(eye);
        m.at(1, 0) = u.x;  m.at(1, 1) = u.y;  m.at(1, 2) = u.z;  m.at(1, 3) = -u.dot(eye);
        m.at(2, 0) = -f.x; m.at(2, 1) = -f.y; m.at(2, 2) = -f.z; m.at(2, 3) = f.dot(eye);
        return m;
    }

    // 행렬 곱셈
    Mat4 operator*(const Mat4& b) const {
        Mat4 result;
        std::memset(result.data, 0, sizeof(result.data));

        for (int col = 0; col < 4; ++col) {
            for (int row = 0; row < 4; ++row) {
                for (int k = 0; k < 4; ++k) {
                    result.at(row, col) += at(row, k) * b.at(k, col);
                }
            }
        }
        return result;
    }

    // 벡터 변환
    Vec4 operator*(const Vec4& v) const {
        return Vec4(
            at(0, 0) * v.x + at(0, 1) * v.y + at(0, 2) * v.z + at(0, 3) * v.w,
            at(1, 0) * v.x + at(1, 1) * v.y + at(1, 2) * v.z + at(1, 3) * v.w,
            at(2, 0) * v.x + at(2, 1) * v.y + at(2, 2) * v.z + at(2, 3) * v.w,
            at(3, 0) * v.x + at(3, 1) * v.y + at(3, 2) * v.z + at(3, 3) * v.w
        );
    }

    Vec3 transformPoint(const Vec3& p) const {
        Vec4 result = (*this) * Vec4(p, 1.0f);
        return result.perspectiveDivide();
    }

    Vec3 transformDirection(const Vec3& d) const {
        Vec4 result = (*this) * Vec4(d, 0.0f);
        return result.toVec3();
    }

    // 전치행렬
    Mat4 transposed() const {
        Mat4 result;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                result.at(i, j) = at(j, i);
            }
        }
        return result;
    }

    // Float 배열로 변환 (JS 전달용 - 이미 column-major)
    const float* toArray() const {
        return data;
    }

    // 배열에서 생성
    static Mat4 fromArray(const float* arr) {
        Mat4 m;
        std::memcpy(m.data, arr, 16 * sizeof(float));
        return m;
    }
};

// ============================================================================
// Quaternion: 회전 표현용 쿼터니언
// ============================================================================
struct Quaternion {
    float x, y, z, w;

    Quaternion() : x(0), y(0), z(0), w(1) {}
    Quaternion(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}

    // 축-각도로 생성
    static Quaternion fromAxisAngle(const Vec3& axis, float angle) {
        float halfAngle = angle * 0.5f;
        float s = std::sin(halfAngle);
        Vec3 n = axis.normalized();
        return Quaternion(n.x * s, n.y * s, n.z * s, std::cos(halfAngle));
    }

    // 오일러각으로 생성 (YXZ 순서)
    static Quaternion fromEuler(float pitch, float yaw, float roll) {
        float cy = std::cos(yaw * 0.5f);
        float sy = std::sin(yaw * 0.5f);
        float cp = std::cos(pitch * 0.5f);
        float sp = std::sin(pitch * 0.5f);
        float cr = std::cos(roll * 0.5f);
        float sr = std::sin(roll * 0.5f);

        return Quaternion(
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy
        );
    }

    Quaternion normalized() const {
        float len = std::sqrt(x * x + y * y + z * z + w * w);
        if (len > 0.0001f) {
            return Quaternion(x / len, y / len, z / len, w / len);
        }
        return Quaternion();
    }

    // 4x4 회전행렬로 변환
    Mat4 toMatrix() const {
        Mat4 m;
        float xx = x * x, yy = y * y, zz = z * z;
        float xy = x * y, xz = x * z, yz = y * z;
        float wx = w * x, wy = w * y, wz = w * z;

        m.at(0, 0) = 1.0f - 2.0f * (yy + zz);
        m.at(0, 1) = 2.0f * (xy - wz);
        m.at(0, 2) = 2.0f * (xz + wy);

        m.at(1, 0) = 2.0f * (xy + wz);
        m.at(1, 1) = 1.0f - 2.0f * (xx + zz);
        m.at(1, 2) = 2.0f * (yz - wx);

        m.at(2, 0) = 2.0f * (xz - wy);
        m.at(2, 1) = 2.0f * (yz + wx);
        m.at(2, 2) = 1.0f - 2.0f * (xx + yy);

        return m;
    }

    std::array<float, 4> toArray() const {
        return {x, y, z, w};
    }
};

} // namespace ar

#endif // MATH_TYPES_H
