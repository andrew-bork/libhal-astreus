#pragma once





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