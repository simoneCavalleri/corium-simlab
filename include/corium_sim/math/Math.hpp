#pragma once

#include <cmath>
#include <array>
#include <algorithm>

namespace corium_sim::math {

constexpr float PI = 3.14159265358979323846f;
constexpr float DEG2RAD = PI / 180.0f;
constexpr float RAD2DEG = 180.0f / PI;

struct Vec2 {
    float x{0.0f};
    float y{0.0f};

    constexpr Vec2() = default;
    constexpr Vec2(float x_, float y_) : x(x_), y(y_) {}
};

struct Vec3 {
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};

    constexpr Vec3() = default;
    constexpr Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    constexpr Vec3 operator+(const Vec3& rhs) const noexcept { return {x + rhs.x, y + rhs.y, z + rhs.z}; }
    constexpr Vec3 operator-(const Vec3& rhs) const noexcept { return {x - rhs.x, y - rhs.y, z - rhs.z}; }
    constexpr Vec3 operator*(float s) const noexcept { return {x * s, y * s, z * s}; }
    constexpr Vec3 operator/(float s) const noexcept { float inv = 1.0f / s; return {x * inv, y * inv, z * inv}; }
    constexpr Vec3 operator-() const noexcept { return {-x, -y, -z}; }

    Vec3& operator+=(const Vec3& rhs) noexcept { x += rhs.x; y += rhs.y; z += rhs.z; return *this; }
    Vec3& operator-=(const Vec3& rhs) noexcept { x -= rhs.x; y -= rhs.y; z -= rhs.z; return *this; }
    Vec3& operator*=(float s) noexcept { x *= s; y *= s; z *= s; return *this; }

    [[nodiscard]] float lengthSq() const noexcept { return x * x + y * y + z * z; }
    [[nodiscard]] float length() const noexcept { return std::sqrt(lengthSq()); }

    [[nodiscard]] Vec3 normalized() const noexcept {
        float len = length();
        if (len > 1e-6f) return *this / len;
        return {0.0f, 0.0f, 0.0f};
    }
};

inline constexpr float dot(const Vec3& a, const Vec3& b) noexcept {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline constexpr Vec3 cross(const Vec3& a, const Vec3& b) noexcept {
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

inline constexpr Vec3 lerp(const Vec3& a, const Vec3& b, float t) noexcept {
    return a * (1.0f - t) + b * t;
}

struct Vec4 {
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};
    float w{1.0f};

    constexpr Vec4() = default;
    constexpr Vec4(float x_, float y_, float z_, float w_ = 1.0f) : x(x_), y(y_), z(z_), w(w_) {}
    constexpr Vec4(const Vec3& v, float w_ = 1.0f) : x(v.x), y(v.y), z(v.z), w(w_) {}
};

/// @brief 4x4 Column-Major Matrix matching WebGPU WGSL mat4x4 layout.
struct alignas(16) Mat4 {
    // Elements stored in column-major order: m[col * 4 + row]
    float m[16]{
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f
    };

    constexpr Mat4() = default;

    static constexpr Mat4 identity() noexcept {
        return Mat4{};
    }

    static constexpr Mat4 zeros() noexcept {
        Mat4 res{};
        for (int i = 0; i < 16; ++i) res.m[i] = 0.0f;
        return res;
    }

    Mat4 operator*(const Mat4& rhs) const noexcept {
        Mat4 res = Mat4::zeros();
        for (int col = 0; col < 4; ++col) {
            for (int row = 0; row < 4; ++row) {
                float sum = 0.0f;
                for (int k = 0; k < 4; ++k) {
                    sum += m[k * 4 + row] * rhs.m[col * 4 + k];
                }
                res.m[col * 4 + row] = sum;
            }
        }
        return res;
    }

    Vec4 operator*(const Vec4& v) const noexcept {
        return Vec4{
            m[0] * v.x + m[4] * v.y + m[8]  * v.z + m[12] * v.w,
            m[1] * v.x + m[5] * v.y + m[9]  * v.z + m[13] * v.w,
            m[2] * v.x + m[6] * v.y + m[10] * v.z + m[14] * v.w,
            m[3] * v.x + m[7] * v.y + m[11] * v.z + m[15] * v.w
        };
    }

    static Mat4 translate(const Vec3& v) noexcept {
        Mat4 res = identity();
        res.m[12] = v.x;
        res.m[13] = v.y;
        res.m[14] = v.z;
        return res;
    }

    static Mat4 scale(const Vec3& s) noexcept {
        Mat4 res = identity();
        res.m[0] = s.x;
        res.m[5] = s.y;
        res.m[10] = s.z;
        return res;
    }

    static Mat4 scale(float s) noexcept {
        return scale(Vec3{s, s, s});
    }

    static Mat4 rotateX(float radians) noexcept {
        Mat4 res = identity();
        float c = std::cos(radians);
        float s = std::sin(radians);
        res.m[5] = c;
        res.m[6] = s;
        res.m[9] = -s;
        res.m[10] = c;
        return res;
    }

    static Mat4 rotateY(float radians) noexcept {
        Mat4 res = identity();
        float c = std::cos(radians);
        float s = std::sin(radians);
        res.m[0] = c;
        res.m[2] = -s;
        res.m[8] = s;
        res.m[10] = c;
        return res;
    }

    static Mat4 rotateZ(float radians) noexcept {
        Mat4 res = identity();
        float c = std::cos(radians);
        float s = std::sin(radians);
        res.m[0] = c;
        res.m[1] = s;
        res.m[4] = -s;
        res.m[5] = c;
        return res;
    }

    /// @brief WebGPU Clip Space Perspective Matrix (Z range: [0, 1])
    static Mat4 perspective(float fovYRadians, float aspectRatio, float zNear, float zFar) noexcept {
        Mat4 res = zeros();
        float tanHalfFov = std::tan(fovYRadians * 0.5f);
        
        res.m[0] = 1.0f / (aspectRatio * tanHalfFov);
        res.m[5] = 1.0f / tanHalfFov;
        res.m[10] = zFar / (zNear - zFar); // WebGPU clip space Z in [0, 1]
        res.m[11] = -1.0f;
        res.m[14] = (zNear * zFar) / (zNear - zFar);
        return res;
    }

    /// @brief Right-Handed LookAt View Matrix
    static Mat4 lookAt(const Vec3& eye, const Vec3& center, const Vec3& up) noexcept {
        Vec3 f = (center - eye).normalized();
        Vec3 s = cross(f, up).normalized();
        Vec3 u = cross(s, f);

        Mat4 res = identity();
        res.m[0] = s.x;
        res.m[4] = s.y;
        res.m[8] = s.z;

        res.m[1] = u.x;
        res.m[5] = u.y;
        res.m[9] = u.z;

        res.m[2] = -f.x;
        res.m[6] = -f.y;
        res.m[10] = -f.z;

        res.m[12] = -dot(s, eye);
        res.m[13] = -dot(u, eye);
        res.m[14] = dot(f, eye);
        return res;
    }
};

} // namespace corium_sim::math
