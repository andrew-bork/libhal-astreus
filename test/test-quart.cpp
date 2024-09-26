#include "math.hpp"
#include <cstdio>
#include <unistd.h>

#define r_t_d 57.2957795131 

int main() {
    math::quarternion q(1.0f);
    // math::quarternion q = math::quarternion::from_euler_ZYX(math::vec3(1.0f, 0.0f, 0.0f));
    // math::quarternion p = math::quarternion::from_euler_ZYX(math::vec3(0.0f, 1.0f, 0.0f));

    math::quarternion k(1.0f, 0.0f, 10.0f*0.01f / r_t_d, 10.0f*0.01f / r_t_d);

    while(true) {
        q = q * k;
        q.norm();
        math::vec3 euler = math::quarternion::to_euler_ZYX(q);
        printf("R: %6.1f° P: %6.1f° Y: %6.1f°\n", euler.x * r_t_d, euler.y * r_t_d, euler.z * r_t_d);
        usleep(10000);
    }

}