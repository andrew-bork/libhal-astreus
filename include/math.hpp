#pragma once

#include "vec3.hpp"
#include <cmath>

namespace math{
    /**
     * @brief quarternions are normalized
     * 
     */
    struct quarternion {
        float w, x, y, z;

        inline quarternion(float p_w=0.0, float p_x=0.0, float p_y=0.0, float p_z=0.0) : w(p_w), x(p_x), y(p_y), z(p_z) {}

        inline quarternion operator+ (const quarternion& r) {
            quarternion res;
            res.w = r.w + w;
            res.x = r.x + x;
            res.y = r.y + y;
            res.z = r.z + z;
            return res;
        }

        inline quarternion operator* (const quarternion& r) {
            quarternion res;
            res.w = w*r.w - x*r.x - y*r.y - z*r.z;
            res.x = r.w*x + r.x*w + r.y*z - r.z*y;
            res.y = r.w*y + r.y*w + r.z*x - r.x*z;
            res.z = r.w*z + r.z*w + r.x*y - r.y*x;
            return res;
        }

        inline quarternion operator* (float r) {
            quarternion res;
            res.w = w*r;
            res.x = x*r;
            res.y = y*r;
            res.z = z*r;
            return res;
        }

        inline void norm () {
            float s = length();
            w /= s;
            x /= s;
            y /= s;
            z /= s;
        }

        inline float length() {
            return sqrt(w*w + x*x + y*y + z*z);
        }


        // static inline float length(const quarternion& q) {
        //     return q.length();
        // }

        static inline quarternion conjugate(const quarternion& q) {
            quarternion res;
            res.w = q.w;
            res.x = -q.x;
            res.y = -q.y;
            res.z = -q.z;
            return res;
            
        }

        static inline quarternion from_angle_axis(float theta, const vec3& axis) {
            quarternion res;
            float c = std::cos(theta/2.0f);
            float s = std::sin(theta/2.0f);
            res.w = c;
            res.x = axis.x*s;
            res.y = axis.y*s;
            res.z = axis.z*s;
            return res;
        }
        static inline quarternion from_euler_ZYX(const vec3& euler) {
            double cy = cos(euler.z*0.5f);
            double sy = sin(euler.z*0.5f);
            double cp = cos(euler.y*0.5f);
            double sp = sin(euler.y*0.5f);
            double cr = cos(euler.x*0.5f);
            double sr = sin(euler.x*0.5f);

            quarternion res;
            res.w = cr*cp*cy + sr*sp*sy;
            res.x = sr*cp*cy - cr*sp*sy;
            res.y = cr*sp*cy + sr*cp*sy;
            res.z = cr*cp*sy - sr*sp*cy;
            return res;
        }
        
        static inline vec3 to_euler_ZYX(const quarternion& q) {
            float  sin_p = 2 * (q.w * q.y - q.z * q.x);
            vec3 out;
            if(sin_p >=1){
                out.x = -2* atan2(q.x, q.w);
                out.y = 1.570796326794897f;
                out.z = 0;
            }else if(sin_p <= -1){
                out.x = 2 * atan2(q.x, q.w);
                out.y = -1.570796326794897f;
                out.z = 0;
            }else {
                out.x = atan2(2*(q.w*q.x+q.y*q.z), 1 - 2 * (q.x*q.x + q.y*q.y));
                out.y = asin(sin_p);
                out.z = atan2(2*(q.w*q.z+q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z));
            }
            return out;
        }

        static vec3 to_angle_axis(const quarternion& q) {
            float s = 1.0f - q.w*q.w;
            float mag = acos(q.w)*2.0f;
            vec3 res;

            if(s < 0.01f && 0.01f < s) {
                res.x = 0.0f;
                res.y = 0.0f;
                res.z = 0.0f;
            }else {
                res.x = q.x*mag/s;
                res.y = q.y*mag/s;
                res.z = q.z*mag/s;
            }
            return res;
        }

        static quarternion exp(const quarternion& q) {
            float theta = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z);
            float c = std::exp(q.w);
            float a = std::cos(theta) * c;
            float b = std::sin(theta) * c / theta;
            quarternion out(a, b * q.x, b * q.y, b * q.z);
            return out;
        }
        static quarternion exp_no_real(const quarternion& q) {
            float theta = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z);
            float a = std::cos(theta);
            float b = std::sin(theta) / theta;
            quarternion out(a, b * q.x, b * q.y, b * q.z);
            return out;
        }

        static quarternion quarternion_between(const vec3& a, const vec3& b) {
            float dot = vec3::dot(a, b);
            quarternion res;
            if(std::abs(dot + 1.0f) < 0.001f) {
                vec3 rotated(a.y, -a.x, a.z);
                res = quarternion::from_angle_axis(3.1415f, rotated);
            }else {
                vec3 cross = vec3::cross(a, b);
                res.w = 1 + dot;
                res.x = cross.x;
                res.y = cross.y;
                res.z = cross.z;
                res.norm();
            }
            return res;
        }
        // static vector rotateVector(math::quarternion& q, math::vector& in);

    };

}