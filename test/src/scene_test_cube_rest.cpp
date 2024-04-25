
#include "scene_test.hpp"
#include <random>

static void init_auxiliary_shape(b3AABB *aabb, b3AuxiliaryShape *auxiliary_shape) {
    auxiliary_shape->set_color(Eigen::RowVector3d(252.0 / 255, 230.0 / 255, 202.0 / 255));
    b3Vector3r min = aabb->min();
    b3Vector3r max = aabb->max();
    auxiliary_shape->add_line(min.x(), min.y(), min.z(), max.x(), min.y(), min.z());
    auxiliary_shape->add_line(max.x(), min.y(), min.z(), max.x(), max.y(), min.z());
    auxiliary_shape->add_line(max.x(), max.y(), min.z(), min.x(), max.y(), min.z());
    auxiliary_shape->add_line(min.x(), max.y(), min.z(), min.x(), min.y(), min.z());
    auxiliary_shape->add_line(min.x(), min.y(), max.z(), max.x(), min.y(), max.z());
    auxiliary_shape->add_line(max.x(), min.y(), max.z(), max.x(), max.y(), max.z());
    auxiliary_shape->add_line(max.x(), max.y(), max.z(), min.x(), max.y(), max.z());
    auxiliary_shape->add_line(min.x(), max.y(), max.z(), min.x(), min.y(), max.z());
    auxiliary_shape->add_line(min.x(), min.y(), min.z(), min.x(), min.y(), max.z());
    auxiliary_shape->add_line(max.x(), min.y(), min.z(), max.x(), min.y(), max.z());
    auxiliary_shape->add_line(max.x(), max.y(), min.z(), max.x(), max.y(), max.z());
    auxiliary_shape->add_line(min.x(), max.y(), min.z(), min.x(), max.y(), max.z());
}


class TestBoxRest: public SceneTestBase {

    int auxiliary_shape_count;
    b3AuxiliaryShape* auxiliary_shape_list;

public:

    TestBoxRest() {

        m_world->set_gravity(b3Vector3r(0, 0, -10));
        {
            b3BodyDef body_def;
            body_def.m_type = b3BodyType::b3_dynamic_body;

            b3Vector3r p(0, 0, 1);
            b3Vector3r q(0, 0, 0);
            b3Vector3r v(0, 0, 0);
            b3Vector3r w(0, 0, 0);

            body_def.set_init_pose(p, q);
            body_def.set_init_velocity(v, w);

            b3Body* cube = m_world->create_body(body_def);

            b3CubeShape cube_shape;
            cube_shape.set_as_box(1.0, 1.0, 1.0);

            b3FixtureDef box_fd;
            box_fd.m_shape = &cube_shape;
            box_fd.m_friction = 0.3;
            box_fd.m_restitution = 1.0;
            box_fd.m_density = 1.0;

            cube->create_fixture(box_fd);
        }

        {
            b3BodyDef body_def;
            body_def.m_type = b3BodyType::b3_dynamic_body;

            b3Vector3r p(0, 0, 3);
            b3Vector3r q(0, 0, 0);
            b3Vector3r v(0, 0, 0);
            b3Vector3r w(0, 0, 0);

            body_def.set_init_pose(p, q);
            body_def.set_init_velocity(v, w);

            b3Body* cube = m_world->create_body(body_def);

            b3CubeShape cube_shape;
            cube_shape.set_as_box(1.0, 1.0, 1.0);

            b3FixtureDef box_fd;
            box_fd.m_shape = &cube_shape;
            box_fd.m_friction = 0.3;
            box_fd.m_restitution = 1.0;
            box_fd.m_density = 1.0;

            cube->create_fixture(box_fd);
        }

        ////////////////////////////////////////////////////


        b3BodyDef ground_bd;
        ground_bd.m_type = b3BodyType::b3_static_body;

        b3Body* ground_body = m_world->create_body(ground_bd);

        b3PlaneShape ground_shape;
        ground_shape.set_as_plane(20, 20);

        b3FixtureDef ground_fd;
        ground_fd.m_shape = &ground_shape;
        ground_fd.m_friction = 0.3;
        ground_fd.m_restitution = 1.0;
        ground_fd.m_density = 0.0;


        ground_body->create_fixture(ground_fd);

    }


    void step() override {

        if (!stop)
            m_world->step(1.0 / 60, 20, 8);

        b3AABB aabb;

        auto* broad_phase = m_world->get_broad_phase();
        auxiliary_shape_count = broad_phase->get_dynamic_tree()->get_node_count();
        int n = broad_phase->get_dynamic_tree()->get_node_capacity();
        int height;

        auxiliary_shape_list = new b3AuxiliaryShape[auxiliary_shape_count];
        for (int i = 1; i < auxiliary_shape_count; ++i) {
            auxiliary_shape_list[i - 1].set_next(auxiliary_shape_list + i);
        }
        int index = 0;

        for (int i = 0; i < n; ++i) {
            broad_phase->get_dynamic_tree()->get_node_info(i, height, aabb);
            if (height != b3_NULL_HEIGHT) {
                init_auxiliary_shape(&aabb, auxiliary_shape_list + index);
                index++;
            }
        }

        b3Contact* c = m_world->get_contact_list();
        while (c) {
            b3Manifold* m = c->get_manifold();
            for (int32 i = 0; i < m->m_point_count; i++) {
                b3ManifoldPoint p = m->m_points[i];
                auxiliary_shape_list->add_point(p.m_local_point.x(), p.m_local_point.y(), p.m_local_point.z());
            }
            c = c->next();
        }
    }


    int get_auxiliary_shape_count() const override {
        return auxiliary_shape_count;
    }

    b3AuxiliaryShape* get_auxiliary_shape_list() const override {
        return auxiliary_shape_list;
    }

    static TestBase* create() {
        return new TestBoxRest;
    }
};

static int test_index = register_test("Cube Scene Test", "Cube Rest", TestBoxRest::create);


