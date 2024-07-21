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

#ifndef BOX3D_B3_CONTACT_CONSTRAINT_HPP
#define BOX3D_B3_CONTACT_CONSTRAINT_HPP


#include "common/b3_types.hpp"
#include "math/b3_mat33.hpp"
#include "math/b3_mat1212.hpp"
#include "collision/b3_collision.hpp"

/**
 * @brief Represents a single point of velocity constraint in contact resolution.
 */
struct b3VelocityConstraintPoint {
    b3Vec3r m_ra; ///< @brief The relative position vector from the center of mass of body A to the contact point.
    b3Vec3r m_rb; ///< @brief The relative position vector from the center of mass of body B to the contact point.
    real m_normal_impulse; ///< @brief The impulse applied in the direction of the contact normal.
    real m_normal_mass; ///< @brief The mass associated with the normal direction.
    real m_bias_velocity; ///< @brief The bias added to the velocity to correct penetration.
    real m_rhs_penetration; ///< @brief The right-hand side value for penetration correction.
    real m_relative_velocity; ///< @brief The relative velocity between the two bodies at the contact point.
    bool m_wait; ///< this value is for situation 4

    real m_tangent1_mass; ///< @brief The mass associated with the tangential1 direction.
    real m_tangent2_mass; ///< @brief The mass associated with the tangential2 direction.
    real m_tangent1_impulse; ///< @brief The impulse applied in the direction of the contact tangent1
    real m_tangent2_impulse; ///< @brief The impulse applied in the direction of the contact tangent2
};

/**
 * @brief Represents the constraints applied to the velocity of contacting bodies.
 */
struct b3ContactVelocityConstraint {

    b3VelocityConstraintPoint m_points[8]; ///< @brief Array of velocity constraint points for multiple contact points.

    b3Vec3r m_normal; ///< @brief The normal vector of the contact.
    b3Vec3r m_tangent1; ///< @brief The tangent1 vector of the contact.
    b3Vec3r m_tangent2; ///< @brief The tangent2 vector of the contact.
    int32 m_index_a; ///< @brief The index of body A in the constraint.
    int32 m_index_b; ///< @brief The index of body B in the constraint.
    real m_inv_mass_a; ///< @brief The inverse mass of body A.
    real m_inv_mass_b; ///< @brief The inverse mass of body B.
    b3Mat33r m_inv_I_a; ///< @brief The inverse inertia tensor of body A.
    b3Mat33r m_inv_I_b; ///< @brief The inverse inertia tensor of body B.
    int32 m_point_count; ///< @brief The number of contact points.
    int32 m_contact_index; ///< @brief The index of the contact in the contact list.
    real m_restitution; ///< @brief The coefficient of restitution for the contact (elasticity).
    real m_restitution_threshold; ///< @brief The threshold below which restitution is applied.
    real m_friction; ///< @brief The coefficient of friction for the contact.
    real** m_JWJT = nullptr; ///< @brief Pointer to a matrix used in the Jacobian transpose calculation.

};

/**
 * @brief Represents the position constraints in contact resolution.
 */
struct b3ContactPositionConstraint
{
    b3Vec3r m_local_points[8]; ///< @brief Local positions of the contact points in the bodies' coordinate spaces.
    b3Vec3r m_local_normal; ///< @brief The local normal vector of the contact.
    b3Vec3r m_local_point; ///< @brief The local position of the contact point.
    int32 m_index_a; ///< @brief The index of body A in the constraint.
    int32 m_index_b; ///< @brief The index of body B in the constraint.
    real m_inv_mass_a; ///< @brief The inverse mass of body A.
    real m_inv_mass_b; ///< @brief The inverse mass of body B.
    b3Vec3r m_center_a; ///< @brief The center of mass of body A.
    b3Vec3r m_center_b; ///< @brief The center of mass of body B.
    b3Vec3r m_local_center_a; ///< @brief The local center of mass of body A.
    b3Vec3r m_local_center_b; ///< @brief The local center of mass of body B.
    b3Mat33r m_inv_I_a; ///< @brief The inverse inertia tensor of body A.
    b3Mat33r m_inv_I_b; ///< @brief The inverse inertia tensor of body B.
    real m_radius_a; ///< @brief The radius of body A.
    real m_radius_b; ///< @brief The radius of body B.
    int32 m_point_count; ///< @brief The number of contact points.
    b3Manifold::Type m_type; ///< @brief The type of manifold for the contact.

};


#endif // BOX3D_B3_CONTACT_CONSTRAINT_HPP
