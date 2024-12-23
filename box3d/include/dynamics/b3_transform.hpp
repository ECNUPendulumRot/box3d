
#ifndef BOX3D_B3_TRANSFORM_HPP
#define BOX3D_B3_TRANSFORM_HPP


#include "common/b3_types.hpp"

#include "math/b3_mat33.hpp"
#include "math/b3_quat.hpp"


template <typename T>
struct b3Transform {

public:

    // The position part of the pose.
    b3Vec3<T> m_p;

    // The rotation matrix of the pose.
    b3Mat33<T> m_r;

    b3Transform() {
        m_p.set_zero();
        m_r.set_identity();
    }

    b3Transform(const b3Transform& other) {
        m_p = other.m_p;
        m_r = other.m_r;
    }

    b3Transform(const b3Vec3<T>& p, const b3Mat33<T>& r) {
        m_p = p;
        m_r = r;
    }

    b3Transform(const b3Vec3<T>& p, const b3Quat<T>& q) {
        m_p = p;
        m_r.set_rotation(q);
    }

    void set(const b3Vec3<T>& p, const b3Quat<T>& q) {
        m_p = p;
        m_r.set_rotation(q);
    }

    void set_position(const b3Vec3<T>& p) {
        m_p = p;
    }

    void set_euler_angles(const b3Vec3<T>& euler) {
        b3Quat<T> q(euler);
        m_r = q.rotation_matrix();
    }

    void set_basis(const b3Mat33r& basis) {
        m_r = basis;
    }

    void set_rotation(const b3Quaternionr& q) {
        m_r.set_rotation(q);
    }

    inline const b3Mat33<T>& rotation_matrix() const {
        return m_r;
    };

    inline b3Vec3<T> transform(const b3Vec3<T>& v) const {
        return m_r * v + m_p;
    }

    inline b3Vec3<T> rotate(const b3Vec3<T>& v) const {
        return m_r * v;
    }

    inline b3Vec3<T> transform_local(const b3Vec3<T>& v) const {
        return m_r.transpose() * (v - m_p);
    }

    b3Transform inverse_times(const b3Transform& t) const {
        b3Vec3 v = m_r.transpose() * (t.position() - m_p);
        b3Mat33 r = m_r.transpose_times(t.m_r);
        b3Quat<T> q;
        r.get_rotation(q);
        return b3Transform(v, q);
    }

    inline b3Vec3<T> position() const {
        return m_p;
    }

    void set_identity() {
        m_r.set_identity();
        m_p.set_zero();
    }

    b3Quat<T> get_rotation() const {
        b3Quat<T> q;
        m_r.get_rotation(q);
        return q;
    }

};

/////////////////////////////////////////////////////////////////////

using b3Transformf = b3Transform<float>;
using b3Transformd = b3Transform<double>;
using b3Transformr = b3Transform<real>;

#endif //BOX3D_B3_TRANSFORM_HPP
