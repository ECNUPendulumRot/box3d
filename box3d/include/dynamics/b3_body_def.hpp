// The MIT License

// Copyright (c) 2024
// Robot Motion and Vision Laboratory at East China Normal University
// Contact: tophill.robotics@gmail.com

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


#ifndef BOX3D_B3_BODY_DEF_HPP
#define BOX3D_B3_BODY_DEF_HPP


#include "dynamics/b3_transform.hpp"

/**
 * @brief  defines the different types of bodies that can exist within a physics simulation.
 */
enum class b3BodyType {

    b3_type_not_defined = -1, ///< Represents an undefined body type.

    b3_static_body = 0, ///< Represents a static body

    b3_kinematic_body = 1, ///< Represents a kinematic body.

    b3_dynamic_body = 2, ///< Represents a dynamic body

    b3_body_type_count = 3 ///< Represents the total count of body types defined in this enumeration.

};

/**
 * @brief The b3BodyDef class serves as a definition or blueprint for creating bodies
 * in a physics simulation.
 */
struct b3BodyDef {

    /**
     * @brief the friend class of b3BodyDef
     */
    friend class b3Body;

    /**
     * @brief the friend class of b3BodyDef
     */
    friend class b3World;

    /**
     * @brief Specifies the type of the body
     */
    b3BodyType m_type = b3BodyType::b3_type_not_defined;

    /**
     * @brief Holds the initial position of the body.
     */
    b3Vec3r m_init_p = b3Vec3r::zero();

    /**
     * @brief Holds the initial orientation (quaternion) of the body.
     */
    b3Quatr m_init_q;

    /**
     * @brief This variable sets the initial speed and direction of the body's movement.
     */
    b3Vec3r m_init_v = b3Vec3r::zero();

    /**
     * @brief Holds the initial angular velocity of the body.
     */
    b3Vec3r m_init_w = b3Vec3r::zero();

public:

    /**
     * @brief constructor of b3BodyDef class
     * @param type The type of the body, defaulting to b3_dynamic_body.
     */
    explicit b3BodyDef(b3BodyType type = b3BodyType::b3_dynamic_body) {
        m_type = type;
    }

    /**
     * @brief Destructor for the b3BodyDef class.
     */
    ~b3BodyDef() = default;

    /**
     * @brief Retrieves the body type.
     * @return The type of the body.
     */
    b3BodyType get_type() const {
        return m_type;
    }

    /**
     * @brief Sets the initial position and orientation of the body.
     * @param position The initial position of the body (b3Vec3r).
     * @param aa  The angle-axis representation of the initial orientation (b3Vec3r).
     */
    inline void set_init_pose(const b3Vec3r& position, const b3Vec3r& aa) {
        m_init_p = position;
        m_init_q = b3_aa_to_quaternion(aa);
    }

    /**
     * @brief Sets the initial linear and angular velocities of the body.
     * @param v The initial linear velocity (b3Vec3r).
     * @param w The initial angular velocity (b3Vec3r).
     */
    inline void set_init_velocity(const b3Vec3r& v, const b3Vec3r& w) {
        m_init_v = v;
        m_init_w = w;
    }
};


#endif //BOX3D_B3_BODY_DEF_HPP
