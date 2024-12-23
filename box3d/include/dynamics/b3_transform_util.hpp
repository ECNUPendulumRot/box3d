
#ifndef B3_TRANSFORM_UTIL_HPP
#define B3_TRANSFORM_UTIL_HPP

#include "b3_transform.hpp"

class b3TransformUtils {

public:

    static void integrate_transform(const b3Transformr& cur_transform, const b3Vec3r& lin_vel, const b3Vec3r& ang_vel, real dt, b3Transformr& predicted_transform) {
        predicted_transform.set_position(cur_transform.position() + lin_vel * dt);

        // 使用指数映射对旋转进行实用参数化
        b3Vec3r axis;
        real f_angle2 = ang_vel.length2();
        real f_angle = 0;
        if (f_angle2 > b3_real_epsilon) {
            f_angle = b3_sqrt(f_angle2);
        }
        // angular不应该过大
        if (f_angle * dt > b3_pi_4) {
            f_angle = b3_pi_4 / dt;
        }
        if (f_angle < real(0.001)) {
            // use Taylor's expansions of sync function
            axis = ang_vel * (real(0.5) * dt - (dt * dt * dt) * (real(0.020833333333)) * f_angle * f_angle);
        } else {
            // sync(fAngle) = sin(c*fAngle)/t
            axis = ang_vel * (b3_sin(real(0.5) * f_angle * dt) / f_angle);
        }
        b3Quaternionr dorn(b3_cos(f_angle * dt * real(0.5)), axis.x, axis.y, axis.z);
        b3Quaternionr orn0 = cur_transform.get_rotation();

        b3Quaternionr predicted_orn = dorn * orn0;
        predicted_orn.safe_normalize();

        if (predicted_orn.length2() > b3_real_epsilon) {
            predicted_transform.set_rotation(predicted_orn);
        } else {
            predicted_transform.set_basis(cur_transform.rotation_matrix());
        }
    }
};


#endif