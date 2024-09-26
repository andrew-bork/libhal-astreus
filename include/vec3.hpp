#pragma once





namespace math {
    struct vec3 {
        float x, y, z;
        constexpr vec3(float p_x=0.0f,float p_y=0.0f,float p_z=0.0f) : x(p_x), y(p_y), z(p_z) {}
    };
};