
#ifndef BOX3D_B3_BODY_HPP
#define BOX3D_B3_BODY_HPP


#include "geometry/b3_mesh.hpp"
#include "dynamics/b3_pose.hpp"

#include "common/b3_allocator.hpp"
#include "dynamics/b3_body_def.hpp"

namespace box3d {

    class b3Body;

    class b3World;

    ////////////////////

    class b3Fixture;
    class b3FixtureDef;
}


class box3d::b3Body {

    /**
     * @brief Type of the body
     */
    b3BodyType m_type;

    /**
     * @brief Mesh of the body
     */
    b3Mesh* m_mesh;

    b3Body* m_next;

    /**
     * @brief The world that the body belongs to
     */
    b3World* m_world;

public:

    /**
     * @brief Construct a new b3Body object
     */
    b3Body():m_mesh(nullptr){
        ;
    };

    b3Fixture* create_fixture(const b3FixtureDef& def);

    virtual void set_mesh(b3Mesh* mesh) {
        m_mesh = mesh;
    };

    inline b3Mesh* mesh() const {
        return m_mesh;
    }

    inline void set_next(b3Body* next) {
        m_next = next;
    }

    inline b3Body* next() const {
        return m_next;
    }

    inline b3BodyType get_type() const {
        return m_type;
    }

    inline void set_type(b3BodyType type) {
        m_type = type;
    }

    inline void set_world(b3World* world) {
        m_world = world;
    }

};


#endif //BOX3D_B3_BODY_HPP
