
#include "scene_test.hpp"
#include "auxiliary_shape.hpp"

#include <random>


void init_auxiliary_shape(b3AABB *aabb, b3AuxiliaryShape *auxiliary_shape) {
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

class TestDropping: public SceneTestBase {

    b3Timer m_timer;

    int count = 0;

    b3AuxiliaryShape* auxiliary_shape_list;

    int auxiliary_shape_count;

public:

    TestDropping() {

        m_world->set_gravity(b3Vector3r(0, 0, -10));

        {
            b3BodyDef body_def;
            body_def.m_type = b3BodyType::b3_dynamic_body;

            for (int i = 0; i < 2; i++) {
                std::random_device rd;
                std::mt19937 gen(rd());
                std::uniform_int_distribution<> distrib_x(-5, 5);
                std::uniform_int_distribution<> distrib_y(-5, 5);
                std::uniform_int_distribution<> distrib_z(5, 10);

                std::uniform_int_distribution<> distrib_w1(-3, 3);
                std::uniform_int_distribution<> distrib_w2(-3, 3);
                std::uniform_int_distribution<> distrib_w3(-3, 3);

                b3Vector3r p({real(distrib_x(gen)), real(distrib_y(gen)), real(distrib_z(gen))});
                b3Vector3r q({real(distrib_w1(gen)), real(distrib_w2(gen)), real(distrib_w3(gen))});

                body_def.set_init_pose(p, q);

                b3Body* cube = m_world->create_body(body_def);

                b3CubeShape cube_shape;
                cube_shape.set_as_box(1.0, 1.0, 1.0);

                b3FixtureDef box_fd;
                box_fd.m_shape = &cube_shape;
                box_fd.m_friction = 0.0;
                box_fd.m_restitution = 1.0;
                box_fd.m_density = 1.0;
                cube->create_fixture(box_fd);
            }


        }

        ////////////////////////////////////////////////////


        b3BodyDef ground_bd;
        ground_bd.m_type = b3BodyType::b3_static_body;

        b3Body* ground_body = m_world->create_body(ground_bd);

        b3PlaneShape ground_shape;
        ground_shape.set_as_plane(20, 20);

        b3FixtureDef ground_fd;
        ground_fd.m_shape = &ground_shape;
        ground_fd.m_friction = 0.0;
        ground_fd.m_restitution = 1.0;
        ground_fd.m_density = 0.0;


        ground_body->create_fixture(ground_fd);
    }

    ~TestDropping() override {
        delete[] auxiliary_shape_list;
    }

    void step() override {
        m_world->step(1.0 / 60, 8, 8);

//        b3AABB aabb;
//
//        auto* broad_phase = m_world->get_broad_phase();
//        auxiliary_shape_count = broad_phase->get_dynamic_tree()->get_node_count();
//        int n = broad_phase->get_dynamic_tree()->get_node_capacity();
//        int height;
//
//        auxiliary_shape_list = new b3AuxiliaryShape[auxiliary_shape_count];
//        for (int i = 1; i < auxiliary_shape_count; ++i) {
//            auxiliary_shape_list[i - 1].set_next(auxiliary_shape_list + i);
//        }
//        int index = 0;
//        for (int i = 0; i < n; ++i) {
//            broad_phase->get_dynamic_tree()->get_node_info(i, height, aabb);
//            if (height != b3_NULL_HEIGHT) {
//                init_auxiliary_shape(&aabb, auxiliary_shape_list + index);
//                index++;
//            }
//        }

    }

//    int get_auxiliary_shape_count() const override {
//        return auxiliary_shape_count;
//    }
//
//    b3AuxiliaryShape* get_auxiliary_shape_list() const override {
//
//        return auxiliary_shape_list;
//    }

    static TestBase* create() {
        return new TestDropping;
    }
};

static int test_index = register_test("Cube Scene Test", "Cube Dropping", TestDropping::create);



