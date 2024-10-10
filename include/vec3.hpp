#pragma once
#include <cmath>




namespace math {
    struct vec3 {
        float x, y, z;
        constexpr vec3(float p_x=0.0f,float p_y=0.0f,float p_z=0.0f) : x(p_x), y(p_y), z(p_z) {}

        constexpr inline vec3 operator+(const vec3& r) {
            vec3 out;
            out.x = x + r.x;
            out.y = y + r.y;
            out.z = z + r.z;
            // return std::move(out);
            return out;
        }
        constexpr inline vec3 operator-(const vec3& r) {
            vec3 out;
            out.x = x - r.x;
            out.y = y - r.y;
            out.z = z - r.z;
            // return std::move(out);
            return out;
        }
        constexpr inline vec3 operator*(float r) {
            vec3 out;
            out.x = x * r;
            out.y = y * r;
            out.z = z * r;
            // return std::move(out);
            return out;
        }
        constexpr inline vec3 operator/(float r) {
            vec3 out;
            out.x = x / r;
            out.y = y / r;
            out.z = z / r;
            // return std::move(out);
            return out;
        }
        constexpr inline vec3& operator+=(const vec3& r) {
            x += r.x;
            y += r.y;
            z += r.z;
            return *this;
        }
        constexpr inline vec3& operator-=(const vec3& r) {
            x -= r.x;
            y -= r.y;
            z -= r.z;
            return *this;
        }
        constexpr inline vec3& operator*=(float r) {
            x *= r;
            y *= r;
            z *= r;
            return *this;
        }

        constexpr inline static vec3 cross(const vec3& l, const vec3& r) {
            vec3 out;
            out.x = l.y * r.z - l.z * r.y;
            out.y = l.z * r.x - l.x * r.z;
            out.z = l.x * r.y - l.y * r.x;
            return out;
        }

        constexpr inline static float length(const vec3& a) {
            return std::sqrt(a.x*a.x+a.y*a.y+a.z*a.z);
        }
        constexpr inline static float dot(const vec3& a, const vec3& b) {
            return a.x*b.x+a.y*b.y+a.z*b.z;
        }
    };
};
constexpr inline math::vec3 operator*(float l, const math::vec3& r) {
    math::vec3 out;
    out.x = r.x * l;
    out.y = r.y * l;
    out.z = r.z * l;
    // return std::move(out);
    return out;
}