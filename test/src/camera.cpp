
#include "camera.hpp"


void Camera::reset_view() {
    m_lookat = {0, 0, 0};
    m_position = {-5, 5, 5};

    build_up_camera_coordinate();
}


void Camera::build_up_camera_coordinate() {
    m_d = (m_position - m_lookat).normalized();
    m_r = b3Vec3f(0, 1, 0).cross(m_d).normalized();
    m_u = m_d.cross(m_r).normalized();
}


void Camera::build_projection_matrix(float* m)
{
    float w = float(m_width);
    float h = float(m_height);
    float ratio = h / w;

    float r = m_s;
    float t = ratio * m_s;

    m[0] = m_n / r;
    m[1] = 0.0f;
    m[2] = 0.0f;
    m[3] = 0.0f;

    m[4] = 0.0f;
    m[5] = m_n / t;
    m[6] = 0.0f;
    m[7] = 0.0f;

    m[8] = 0.0f;
    m[9] = 0.0f;
    m[10] =-(m_f + m_n) / (m_f - m_n);
    m[11] = -1;

    m[12] = 0.0f;
    m[13] = 0.0f;
    m[14] = -2.0f * m_f * m_n / (m_f - m_n);
    m[15] = 0.0f;
}


void Camera::build_view_matrix(float *v) const {

    v[0] = m_r.x;
    v[1] = m_u.x;
    v[2] = m_d.x;
    v[3] = 0.0f;

    v[4] = m_r.y;
    v[5] = m_u.y;
    v[6] = m_d.y;
    v[7] = 0.0f;

    v[8] = m_r.z;
    v[9] = m_u.z;
    v[10] = m_d.z;
    v[11] = 0.0f;

    v[12] = -m_r.dot(m_position);
    v[13] = -m_u.dot(m_position);
    v[14] = -m_d.dot(m_position);
    v[15] = 1.0f;
}



