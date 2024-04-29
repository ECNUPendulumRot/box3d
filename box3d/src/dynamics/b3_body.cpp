
#include "dynamics/b3_body.hpp"

#include "dynamics/b3_world.hpp"
#include "dynamics/b3_body_def.hpp"
#include "dynamics/b3_mass_property.hpp"

#include "collision/b3_fixture.hpp"

///////////  free the shape memory space belong to fixture //////////////
#include "geometry/b3_cube_shape.hpp"
#include "geometry/b3_sphere_shape.hpp"
#include "geometry/b3_plane_shape.hpp"


b3Body::b3Body(const b3BodyDef &body_def): m_volume(0.0), m_inertia(b3Matrix3r::zero())
{
    m_type = body_def.m_type;
    m_p = body_def.m_init_p;
    m_q = body_def.m_init_q;
    m_v = body_def.m_init_v;
    m_w = body_def.m_init_w;
    m_density = body_def.m_density;
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
    }

    // After create a new fixture, may cause new contacts in the scene
    m_world->awake_contact_check();

    return fixture;
}


void b3Body::reset_mass_data()
{
    m_mass = 0.0;
    m_inv_mass = 0.0;
    m_inertia = b3Matrix3r::zero();
    m_inv_inertia = b3Matrix3r::zero();

    b3_assert(m_type == b3BodyType::b3_dynamic_body);

    b3Vector3r local_center = b3Vector3r::zero();

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

        b3Vector3r col1(bxx, -bxy, -bxz);
        b3Vector3r col2(-bxy, byy, -byz);
        b3Vector3r col3(-bxz, -byz, bzz);
        b3Matrix3r bias_center(col1, col2, col3);

        m_inertia -= m_mass * bias_center;

        b3_assert(m_inertia.determinant() > 0.0);

        m_inv_inertia = m_inertia.inverse();

    } else {
        m_inv_inertia = b3Matrix3r::zero();
        m_inv_inertia = b3Matrix3r::zero();
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
        }
        m_world->get_block_allocator()->free(destroy_fixture, sizeof(b3Fixture));
    }
}


real b3Body::kinetic_energy() const {

    // kinetic energy = 1/2 * m * v^2 + 1/2 * (I1 * w1^2 + I2 * w2^2 + I3 * w3^2)
    return real(0.5) * (m_mass * m_v.length2() + (m_w * m_inertia).dot(m_w));

}


