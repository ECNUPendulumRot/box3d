
#ifndef B3_SOLVER_BODY_HPP
#define B3_SOLVER_BODY_HPP

#include "dynamics/b3_transform.hpp"
#include "dynamics/b3_transform_util.hpp"

#include <iostream>

class b3Body;

struct b3SolverBody {
    b3Transformr m_world_transform;
    b3Vec3r m_delta_linear_velocity;
    b3Vec3r m_delta_angular_velocity;

    b3Vec3r m_inv_mass;
    b3Vec3r m_push_velocity;
    b3Vec3r m_turn_velocity;
    b3Vec3r m_linear_velocity;
    b3Vec3r m_angular_velocity;
    b3Vec3r m_external_force_impulse;
    b3Vec3r m_external_torque_impulse;

    b3Body* m_original_body;

    void set_world_transform(const b3Transformr& world_transform) {
        m_world_transform = world_transform;
    }

    const b3Transformr& get_world_transform() const
    {
        return m_world_transform;
    }

    void get_velocity_in_local_point_no_delta(const b3Vec3r& rel_pos, b3Vec3r& velocity) const {

        if (m_original_body) {
            velocity = m_linear_velocity + m_external_force_impulse + (m_angular_velocity + m_external_torque_impulse).cross(rel_pos);
        } else {
            velocity.set_zero();
        }
    }

    void get_angular_velocity(b3Vec3r& angular_velocity) const {
        if (m_original_body) {
            angular_velocity = m_angular_velocity + m_delta_angular_velocity;
        } else {
            angular_velocity.set_zero();
        }
    }

    void apply_impulse(const b3Vec3r& linear_component, const b3Vec3r& angular_component, const real impulse) {
        if (m_original_body) {
            m_delta_linear_velocity += impulse * linear_component;
            m_delta_angular_velocity += impulse * angular_component;
        }
    }

    void internal_apply_push_impulse(const b3Vec3r& linear_component, const b3Vec3r& angular_component, const real impulse) {
        if (m_original_body) {
            m_push_velocity += impulse * linear_component;
            m_turn_velocity += impulse * angular_component;
        }
    }

    const b3Vec3r& get_delta_linear_velocity() const {
        return m_delta_linear_velocity;
    }

    const b3Vec3r& get_delta_angular_velocity() const {
        return m_delta_angular_velocity;
    }

    const b3Vec3r& get_push_velocity() const {
        return m_push_velocity;
    }

    const b3Vec3r& get_turn_velocity() const {
        return m_turn_velocity;
    }

    b3Vec3r& internal_get_delta_linear_velocity() {
        return m_delta_linear_velocity;
    }

    b3Vec3r& internal_get_delta_angular_velocity() {
        return m_delta_angular_velocity;
    }

    const b3Vec3r& internal_get_inv_mass() const {
        return m_inv_mass;
    }

    void internal_set_inv_mass(const b3Vec3r& inv_mass) {
        m_inv_mass = inv_mass;
    }

    b3Vec3r& internal_get_push_velocity() {
        return m_push_velocity;
    }

    b3Vec3r& internal_get_turn_velocity() {
        return m_turn_velocity;
    }

    void internal_get_velocity_in_local_point_obsolete(const b3Vec3r& rel_pos, b3Vec3r& velocity) const {
        velocity = m_linear_velocity + m_delta_linear_velocity + (m_angular_velocity + m_delta_angular_velocity).cross(rel_pos);
    }

    void internal_get_angular_velocity(b3Vec3r& velocity) const {
        velocity = m_angular_velocity + m_delta_angular_velocity;
    }

    void internal_apply_impulse(const b3Vec3r& linear_component, const b3Vec3r& angular_component, const real impulse) {
        if (m_original_body) {
            m_delta_linear_velocity += impulse * linear_component;
            m_delta_angular_velocity += impulse * angular_component;

//            std::cout << "impulse: " << impulse << std::endl;
//            std::cout << "delta linear: " << m_delta_linear_velocity.x << " " << m_delta_linear_velocity.y << " " << m_delta_linear_velocity.z << std::endl;
//            std::cout << "delta angular: " << m_delta_angular_velocity.x << " " << m_delta_angular_velocity.y << " " << m_delta_angular_velocity.z << std::endl;
        }
    }

    void write_back_velocity() {
        if (m_original_body) {
            m_linear_velocity += m_delta_linear_velocity;
            m_angular_velocity += m_delta_angular_velocity;
        }
    }

    // TODO: can delete split_impulse_turn_erp?
    void write_back_velocity_and_transform(real dt, real split_impulse_turn_erp = 0.1) {

        if (m_original_body) {
            m_linear_velocity += m_delta_linear_velocity;
            m_angular_velocity += m_delta_angular_velocity;

//            std::cout << "delta linear: " << m_delta_linear_velocity.x << " " << m_delta_linear_velocity.y << " " << m_delta_linear_velocity.z << std::endl;
//            std::cout << "delta angular: " << m_delta_angular_velocity.x << " " << m_delta_angular_velocity.y << " " << m_delta_angular_velocity.z << std::endl;

            b3Transformr new_transform;
            if (!m_push_velocity.is_zero() || !m_turn_velocity.is_zero()) {
                b3TransformUtils::integrate_transform(m_world_transform, m_push_velocity, m_turn_velocity * split_impulse_turn_erp, dt, new_transform);
                m_world_transform = new_transform;
            }
        }
    }
};

#endif