
#ifndef BOX3D_B3_POSE_HPP
#define BOX3D_B3_POSE_HPP


#include "common/b3_types.hpp"

namespace box3d {
    template <typename T>
    class b3Pose;
}


template <typename T>
class box3d::b3Pose {

    /**
     * The position part of the pose relative to m_rel_p.
     */
    b3Vector3<T> m_p;

    /**
     * The orientation part of the pose relative to m_rel_p.
     */
    b3Vector3<T> m_r;

    /**
     * The relative pose of this pose.
     * If this is a nullptr, then just relative to world frame
     */
    b3Pose<T>* m_rel_p;

public:

    /**
     * @brief Construct a new b3Pose object
     */
    b3Pose();

    /**
     * @brief Construct a new b3Pose object
     * @param x: The x position
     * @param y: The y position
     * @param z: The z position
     * @param r_x: The x rotation theta
     * @param r_y: The y rotation theta
     * @param r_z: The z rotation theta
     */
    b3Pose(const T& x,
           const T& y,
           const T& z,
           const T& r_x,
           const T& r_y,
           const T& r_z);

    inline void set_relative_pose(b3Pose<T>* rel_pose){
        m_rel_p = rel_pose;
    };

    inline void set_position(b3Vector3<T> p){
        m_p = p;
    };

    inline void set_position(const T& x, const T& y, const T& z){
        m_p = b3Vector3<T>(x, y, z);
    };

    inline void set_rotation(b3Vector3<T> p){
        m_r = p;
    };

    inline void set_rotation(const T& x, const T& y, const T& z){
        m_r = b3Vector3<T>(x, y, z);
    };

};

using b3PoseF = box3d::b3Pose<float>;
using b3PoseD = box3d::b3Pose<double>;

#endif //BOX3D_B3_POSE_HPP
