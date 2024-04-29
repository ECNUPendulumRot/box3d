
#ifndef BOX3D_CAMERA_HPP
#define BOX3D_CAMERA_HPP

#include "box3d.hpp"


struct Camera
{
    Camera() {
        m_width = 1280;
        m_height = 800;

        reset_view();
    }

    void reset_view();

    void build_up_camera_coordinate();

    void build_projection_matrix(float* m);

    void build_view_matrix(float* v) const;

    void build_model_matrix(float* m) const;

    int32 m_width;
    int32 m_height;

    b3Vec3f m_position;
    b3Vec3f m_lookat;

    float m_s = 0.1f;
    float m_n = 0.1f;
    float m_f = 100.0f;

    b3Vec3f m_d;
    b3Vec3f m_u;
    b3Vec3f m_r;
};


#endif //BOX3D_CAMERA_HPP
