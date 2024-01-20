
#ifndef BOX3D_B3_TRANSFORM_HPP
#define BOX3D_B3_TRANSFORM_HPP


#include <Eigen/Geometry>

#include "common/b3_types.hpp"

#include "math/b3_matrix.hpp"


template <typename T>
class b3Transform {

public:

    /**
     * The position part of the pose.
     */
    b3Vector3<T> m_p;

    /**
     * The orientation part of the pose.
     */
    b3Vector3<T> m_r;

    /**
     * The rotation matrix of the pose.
     */
    b3Matrix3<T> m_r_t;

    /**
     * @brief Construct a new b3Transform object
     */
    b3Transform();

    b3Transform(const b3Transform& other) {
        m_p = other.m_p;
        m_r = other.m_r;
    }

    /**
     * @brief Construct a new b3Transform object
     * @param x: The x position
     * @param y: The y position
     * @param z: The z position
     * @param r_x: The x rotation theta
     * @param r_y: The y rotation theta
     * @param r_z: The z rotation theta
     */
    b3Transform(const T& x,
                const T& y,
                const T& z,
                const T& r_x,
                const T& r_y,
                const T& r_z);

    inline void set_linear(b3Vector3<T> p){
        m_p = p;
    };

    inline void set_linear(const T& x, const T& y, const T& z){
        m_p = b3Vector3<T>(x, y, z);
    };

    inline void set_angular(b3Vector3<T> p){
        m_r = p;
        m_r_t = rotation_matrix();
    };

    inline void set_angular(const T& x, const T& y, const T& z){
        m_r = b3Vector3<T>(x, y, z);
        m_r_t = rotation_matrix();
    };

    inline b3Vector3<T> linear() const {
        return m_p;
    };

    inline b3Vector3<T> angular() const {
        return m_r;
    };

    inline b3Matrix3<T> rotation_matrix_b3() const {
        return m_r_t;
    };

    inline b3Matrix3<T> rotation_matrix() const;


    static b3Transform<T> zero() {
        return b3Transform<T>(T(0), T(0), T(0), T(0), T(0), T(0));
    };

    inline b3Vector3<T> transform(const b3Vector3<T>& v) const {
        return m_r_t * v + m_p;
    }

    inline b3Vector3<T> transform_local(const b3Vector3<T>& v) const {
        return m_r_t.transpose() * (v - m_p);
    }

};


/////////////////////////////////////////////////////////////////////


template <typename T>
b3Transform<T>::b3Transform()
{
    m_p.set_zero();
    m_r.set_zero();
}


template<typename T>
b3Transform<T>::b3Transform(const T &x,
                            const T &y,
                            const T &z,
                            const T &r_x,
                            const T &r_y,
                            const T &r_z):
    m_p(b3Vector3<T>(x,   y,   z)),
    m_r(b3Vector3<T>(r_x, r_y, r_z))
{
    ;
}


template<typename T>
b3Matrix3<T> b3Transform<T>::rotation_matrix() const {
    if (m_r.is_zero()) {
        return b3Matrix3<T>::identity();
    }

    // Rodrigues' rotation formula:
    // R = I + sin(theta) * K + (1 - cos(theta)) * K^2
    T l = m_r.length();
    b3Vector3<T> v = m_r / l;

    b3Matrix3<T> res;
    b3Vector3<T> sin_axis = sin(l) * v;
    T c = cos(l);
    b3Vector3<T> cos1_axis = (T(1) - c) * v;

    T tmp = cos1_axis.x() * v.y();
    res(0, 1) = tmp - sin_axis.z();
    res(1, 0) = tmp + sin_axis.z();

    tmp = cos1_axis.x() * v.z();
    res(0, 2) = tmp + sin_axis.y();
    res(2, 0) = tmp - sin_axis.y();

    tmp = cos1_axis.y() * v.z();
    res(1, 2) = tmp - sin_axis.x();
    res(2, 1) = tmp + sin_axis.x();

    cos1_axis = cos1_axis.cwise_product(v);
    res(0, 0) = cos1_axis.x() + c;
    res(1, 1) = cos1_axis.y() + c;
    res(2, 2) = cos1_axis.z() + c;

    return res;
}


using b3TransformF = b3Transform<float>;
using b3TransformD = b3Transform<double>;


#endif //BOX3D_B3_TRANSFORM_HPP
