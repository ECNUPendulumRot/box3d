
#include "dynamics/b3_body.hpp"

#include "dynamics/b3_world.hpp"
#include "dynamics/b3_body_def.hpp"
#include "dynamics/b3_mass_property.hpp"

#include "collision/b3_fixture.hpp"

///////////  free the shape memory space belong to fixture //////////////
#include "geometry/b3_cube_shape.hpp"
#include "geometry/b3_sphere_shape.hpp"
#include "geometry/b3_plane_shape.hpp"
#include "geometry/b3_cone_shape.hpp"
#include "geometry/b3_cylinder_shape.hpp"

#include "dynamics/constraint/b3_constraint_base.hpp"

b3Body::b3Body(const b3BodyDef &body_def): m_volume(0.0), m_inertia(b3Mat33r::zero())
{
    m_type = body_def.m_type;
    m_p = body_def.m_init_p;
    m_q = body_def.m_init_q;
    m_v = body_def.m_init_v;
    m_w = body_def.m_init_w;
    m_density = body_def.m_density;
    m_linear_damping = body_def.m_linear_damping;
    m_angular_damping = body_def.m_angular_damping;

    m_world_transform.set(m_p, m_q);
}


void b3Body::set_gravity_scale(real scale)
{
    m_gravity_scale = scale;
}

b3Vec3r b3Body::get_gravity() const {
    return m_gravity_scale * m_world->get_gravity() * m_mass;
}

b3Fixture* b3Body::create_fixture(const b3FixtureDef &def)
{
    b3_assert(m_world != nullptr);

    void *memory = m_world->get_block_allocator()->allocate(sizeof(b3Fixture));
    auto *fixture = new(memory) b3Fixture;

    // In this line, the proxies are allocated, but not assigned
    fixture->create_fixture(m_world->get_block_allocator(), def, this);

    // add shape pointer in the class world
    m_world->add_shape(fixture->get_shape());

    // create proxies for the fixture
    auto *broad_phase = m_world->get_broad_phase();

    b3Transformr xf(m_p, m_q);
    fixture->create_proxy(broad_phase, xf);
    fixture->m_body = this;

    // adjust the fixture linked list
    fixture->m_next = m_fixture_list;
    m_fixture_list = fixture;
    ++m_fixture_count;

    if (fixture->m_density > 0.0) {
        reset_mass_data();
        update_inertia_tensor();
    }

    // After create a new fixture, may cause new contacts in the scene
    m_world->awake_contact_check();

    return fixture;
}

bool b3Body::add_constraint(b3ConstraintBase* constraint) {
    if (find_constraint(constraint) == -1) {
        m_constraints.push_back(constraint);
        return true;
    }
    return false;
}

int b3Body::find_constraint(b3ConstraintBase* constraint) {
    for (int i = 0; i < m_constraints.size(); i++) {
        if (m_constraints[i] == constraint) {
            return i;
        }
    }
    return -1;
}

bool b3Body::remove_constraint(b3ConstraintBase* constraint) {
    int find_index = find_constraint(constraint);
    if (find_index != -1) {
        m_constraints.erase(m_constraints.begin() + find_index);
        return true;
    }
    return false;
}

bool b3Body::should_collide(const b3Body* other) {

    // At least one body should be dynamic
    if (m_type != b3BodyType::b3_dynamic_body && other->m_type != b3BodyType::b3_dynamic_body) {
        return false;
    }

    for (b3ConstraintBase* constraint : m_constraints) {
        if (constraint->get_bodyA() == other || constraint->get_bodyB() == other) {
            if (!constraint->get_collide_connected()) {
                return false;
            }
        }
    }
    return true;
}


b3Mat33r b3Body::get_local_inertia() const {
//    b3Vec3r local_inertia;
//    const b3Vec3r inertia = m_local_inv_inertia;
//    local_inertia.set(
//        inertia.x != real(0.0) ? real(1.0) / inertia.x : real(0.0),
//        inertia.y != real(0.0) ? real(1.0) / inertia.y : real(0.0),
//        inertia.z != real(0.0) ? real(1.0) / inertia.z : real(0.0)
//        );
//    return local_inertia;
    return m_local_inertia;
}

// https://www.cnblogs.com/21207-iHome/p/9196997.html
b3Vec3r b3Body::compute_gyro_scopic_implicit(real dt) const {

    b3Mat33r Ib = get_local_inertia();
    // b3Vec3r idl = get_local_inertia();
    b3Vec3r omega1 = m_w;
    b3Quaternionr q = m_world_transform.get_rotation();
    // Convert to body coordinates
    b3Vec3r omegab = quat_rotate(q.inverse(), omega1);

    // b3Mat33r Ib(idl.x, 0, 0, 0, idl.y, 0, 0, 0, idl.z);

    b3Vec3r ibo = Ib * omegab;

    // Residual vector
    b3Vec3r f = dt * omegab.cross(ibo);

    b3Mat33r skew0;
    skew0.set_skew_symmetric_matrix(omegab);
    b3Vec3r om = Ib * omegab;
    b3Mat33r skew1;
    skew1.set_skew_symmetric_matrix(om);

    // Jacobian

    b3Mat33r J = Ib + (skew0 * Ib - skew1) * dt;

    b3Vec3r omega_div = J.solve33(f);

    // Single Netwon-Raphson update
    omegab = omegab - omega_div;
    // Back to world coordinates
    b3Vec3r omega2 = quat_rotate(q, omegab);
    b3Vec3r gf = omega2 - omega1;
    // spdlog::info("fg:: {}, {}, {}", gf.x, gf.y, gf.z);
    return gf;
}

void b3Body::update_inertia_tensor() {
    // m_inv_inertia_tensor_world = m_world_transform.rotation_matrix().scaled(m_local_inv_inertia) * m_world_transform.rotation_matrix().transpose();
    m_inv_inertia_tensor_world = m_world_transform.rotation_matrix() * m_local_inv_inertia * m_world_transform.rotation_matrix().transpose();
}

void b3Body::apply_gravity() {
    m_force += get_gravity();
}

#include <iostream>

void b3Body::reset_mass_data()
{
    m_mass = 0.0;
    m_inv_mass = 0.0;
    m_inertia = b3Mat33r::zero();
    m_inv_inertia = b3Mat33r::zero();
    m_local_inertia = b3Mat33r::zero();
    m_local_inv_inertia = b3Mat33r::zero();

    b3_assert(m_type == b3BodyType::b3_dynamic_body);

    b3Vec3r local_center = b3Vec3r::zero();

    // see Mirtich's paper, Fast and Accurate Computation of Polyhedral Mass Properties
    for (b3Fixture *f = m_fixture_list; f != nullptr; f = f->m_next) {
        if (f->m_density == 0.0) {
            continue;
        }

        b3MassProperty mass_data;
        f->get_mass_data(mass_data);
        m_mass += mass_data.m_mass;

        b3Transformr xf = f->get_local_transform();

        local_center += mass_data.m_mass * (mass_data.m_center + xf.position());
        m_local_inertia += xf.rotation_matrix().transpose() * mass_data.m_local_Inertia * xf.rotation_matrix();
    }

    std::cout << "local inertia: " << m_local_inertia(0, 0) << ", " << m_local_inertia(0, 1) << ", " << m_local_inertia(0, 2) << ", "
                                    << m_local_inertia(1, 0) << ", " << m_local_inertia(1, 1) << ", " << m_local_inertia(1, 2) << ", "
                                    << m_local_inertia(2, 0) << ", " << m_local_inertia(2, 1) << ", " << m_local_inertia(2, 2) << std::endl;

    if (m_mass > 0.0) {
        m_inv_mass = 1.0 / m_mass;
        local_center *= m_inv_mass;
    }

    if (m_local_inertia.determinant() > 0) {
        real bxx = local_center.y * local_center.y + local_center.z * local_center.z;
        real byy = local_center.x * local_center.x + local_center.z * local_center.z;
        real bzz = local_center.x * local_center.x + local_center.y * local_center.y;
        real bxy = local_center.x * local_center.y;
        real bxz = local_center.x * local_center.z;
        real byz = local_center.y * local_center.z;

        b3Vec3r col1(bxx, -bxy, -bxz);
        b3Vec3r col2(-bxy, byy, -byz);
        b3Vec3r col3(-bxz, -byz, bzz);
        b3Mat33r bias_center(col1, col2, col3);

        // ????
        // m_local_inertia -= m_mass * bias_center;
        m_local_inertia += m_mass * bias_center;

        b3_assert(m_local_inertia.determinant() > 0);

        m_local_inv_inertia = m_local_inertia.inverse();
    } else {
        m_local_inertia.set_zero();
        m_local_inv_inertia.set_zero();
    }

    // TODO: check whether m_sweep is needed
    // m_local_center = m_xf.transform(local_center);
}

void b3Body::synchronize_fixtures()
{
    b3BroadPhase *broad_phase = m_world->get_broad_phase();

    // if this body is awake ?

    // in box2d, we will get two xf: xf1 and xf2
    // xf1 is the position at the begin of the frame
    // xf2 is the position at the end of the frame
    // TODO: check if we need to do this.
    b3Transformr xf(m_p, m_q);
    for(b3Fixture* f = m_fixture_list; f; f = f->m_next) {
        f->synchronize(broad_phase, xf, xf);
    }
}


void b3Body::destroy_fixtures() {
    b3_assert(m_world != nullptr);

    // TODO: Check this function
    while (m_fixture_list) {
        b3Fixture *destroy_fixture = m_fixture_list;
        m_fixture_list = m_fixture_list->m_next;

        b3Shape *destroy_shape = destroy_fixture->get_shape();

        if (destroy_shape->get_type() == b3ShapeType::e_sphere) {
            m_world->get_block_allocator()->free(destroy_shape, sizeof(b3SphereShape));
        } else if (destroy_shape->get_type() == b3ShapeType::e_cube) {
            m_world->get_block_allocator()->free(destroy_shape, sizeof(b3CubeShape));
        } else if (destroy_shape->get_type() == b3ShapeType::e_plane) {
            m_world->get_block_allocator()->free(destroy_shape, sizeof(b3PlaneShape));
            // TODO: deal with other situation
        } else if (destroy_shape->get_type() == b3ShapeType::e_cone) {
            m_world->get_block_allocator()->free(destroy_shape, sizeof(b3ConeShape));
        } else if (destroy_shape->get_type() == b3ShapeType::e_cylinder) {
            m_world->get_block_allocator()->free(destroy_shape, sizeof(b3CylinderShape));
        }
        m_world->get_block_allocator()->free(destroy_fixture, sizeof(b3Fixture));
    }
}


real b3Body::kinetic_energy() const {
    // kinetic energy = 1/2 * m * v^2 + 1/2 * (I1 * w1^2 + I2 * w2^2 + I3 * w3^2)
    return real(0.5) * (m_mass * m_v.length2() + (m_w * m_inertia).dot(m_w));
}


