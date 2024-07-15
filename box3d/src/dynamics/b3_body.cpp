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


#include "dynamics/b3_body.hpp"

#include "dynamics/b3_world.hpp"
#include "dynamics/b3_body_def.hpp"
#include "dynamics/b3_mass_property.hpp"

#include "collision/b3_fixture.hpp"

///////////  free the shape memory space belong to fixture //////////////
#include "geometry/b3_cube_shape.hpp"
#include "geometry/b3_sphere_shape.hpp"
#include "geometry/b3_plane_shape.hpp"

/**
 * @brief  Constructor that initializes the body with a definition.
 * @param body_def A reference to a b3BodyDef structure that contains the initial
 * configuration for the body.
 */
b3Body::b3Body(const b3BodyDef &body_def): m_volume(0.0), m_inertia(b3Mat33r::zero())
{

    m_flags |= e_awake_flag;

    m_type = body_def.m_type;
    m_p = body_def.m_init_p;
    m_q = body_def.m_init_q;
    m_v = body_def.m_init_v;
    m_w = body_def.m_init_w;

    m_local_center = b3Vec3r::zero();
    m_sweep.p0 = m_p;
    m_sweep.p = m_p;
    m_sweep.q0 = m_q;
    m_sweep.q = m_q;
    m_sweep.alpha0 = 0.0;
}

/**
 * @brief The function creates and attaches a fixture to the body, defining the
 * shape and physical properties of the body.
 * @param def  A reference to a b3FixtureDef structure that contains the configuration
 * for the fixture
 * @return Returns a pointer to the newly created b3Fixture object.
 */
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

    b3Transr xf(m_p, m_q);
    fixture->create_proxy(broad_phase, xf);
    fixture->m_body = this;

    // adjust the fixture linked list
    fixture->m_next = m_fixture_list;
    m_fixture_list = fixture;
    ++m_fixture_count;

    if (fixture->m_density > 0.0) {
  	    reset_mass_data();
    }



    // After create a new fixture, may cause new contacts in the scene
    m_world->awake_contact_check();

    return fixture;
}

/**
 * @brief recalculate mass, inertia, and center of mass of the body.
 */
void b3Body::reset_mass_data()
{
    m_mass = 0.0;
    m_inv_mass = 0.0;
    m_inertia = b3Mat33r::zero();
    m_inv_inertia = b3Mat33r::zero();
    m_local_center = b3Vec3r::zero();

    if (m_type == b3BodyType::b3_static_body || m_type == b3BodyType::b3_kinematic_body) {
        m_sweep.p0 = m_p;
        m_sweep.p = m_p;
        m_sweep.q0 = m_q;
        return;
    }

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

        local_center += mass_data.m_mass * mass_data.m_center;
        m_inertia += mass_data.m_Inertia;
    }

    if (m_mass > 0.0) {
        m_inv_mass = 1.0 / m_mass;
        local_center *= m_inv_mass;
    }

    if (m_inertia.determinant() > 0) {
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

        m_inertia -= m_mass * bias_center;

        b3_assert(m_inertia.determinant() > 0.0);

        m_inv_inertia = m_inertia.inverse();

    } else {
        m_inv_inertia = b3Mat33r::zero();
        m_inv_inertia = b3Mat33r::zero();
    }

    m_local_center = local_center;
    m_sweep.m_local_center = local_center;
    // TODO: use transformation instead of directly assign
    // get the center of mass in sweep
    b3Transr xf(m_p, m_q);
    m_sweep.p0 = m_sweep.p = xf.transform(m_local_center);
    // TODO: if object added dynamically, the velocity need to be updated
}

/**
 * @brief get velocity and position of bodies in the world at the time step begin.
 * solve velocity constraints(if exist), integrate positions.
 * and then, write the position and velocity back to the body
 */
void b3Body::synchronize_fixtures()
{
    b3BroadPhase *broad_phase = m_world->get_broad_phase();

    // if this body is awake ?

    // in box2d, we will get two xf: xf1 and xf2
    // xf1 is the position at the begin of the frame
    // xf2 is the position at the end of the frame
    // TODO: check if we need to do this.
    b3Transr xf(m_p, m_q);
    for(b3Fixture* f = m_fixture_list; f; f = f->m_next) {
        f->synchronize(broad_phase, xf, xf);
    }
}

/**
 * @brief responsible for deallocating all fixtures associated with the b3Body
 * object, including their shapes, from the physics world.
 */
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
        }
        m_world->get_block_allocator()->free(destroy_fixture, sizeof(b3Fixture));
    }
}

/**
 * @brief The advance function advances the sweep to a given interpolation factor alpha.
 * @param alpha The interpolation factor to advance
 */
void b3Sweep::advance(real alpha)
{
    b3_assert(alpha0 < 1.0);

    real beta = (alpha - alpha0) / (real(1.0) - alpha0);

    p0 += beta * (p - p0);
    q0 += (beta * (q - q0));
    q0.normalize();
    alpha0 = alpha;
}

/**
 * @brief calculates the interpolated transformation of an object based on a
 * given interpolation factor beta.
 * @param xf This is a reference to a b3Transr object that will hold the
 * interpolated position and orientation after the function executes.
 * @param beta The interpolation factor to calculate the transformation.
 */
void b3Sweep::get_transform(b3Transr &xf, real beta) const
{
    xf.m_p = (real(1.0) - beta) * p0 + beta * p;

    b3Quatr quat = (real(1.0) - beta) * q0 + beta * q;
    quat.normalize();
    xf.set_quaternion(quat);
    xf.m_p -= xf.rotate(m_local_center);
}
