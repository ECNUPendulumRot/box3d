
#ifndef BOX3D_B3_WORLD_HPP
#define BOX3D_B3_WORLD_HPP

#include <filesystem>
#include <vector>

#include "dynamics/b3_body.hpp"
#include "dynamics/b3_body_def.hpp"

#include "geometry/b3_shape.hpp"

#include "collision/b3_broad_phase.hpp"
#include "collision/b3_contact_manager.hpp"

#include "dynamics/b3_island.hpp"

#include "common/b3_block_allocator.hpp"


class b3World {

    b3Body* m_body_list;

    b3Shape* m_shape_list;

    int32 m_shape_count;

    int32 m_body_count;

    b3Vector3d m_gravity = b3Vector3d(0, 0, 0);

    double m_hz = 60;

    b3BlockAllocator m_block_allocator;

    b3ContactManager m_contact_manager;

    bool m_new_contacts = false;

    std::vector<b3Island*> m_island_list;

public:

    b3World();

    ~b3World();

    // TODO: implement this
    b3Body* create_body(const b3BodyDef& def);

    inline bool empty() const {
        return m_body_count == 0;
    }

    void test_step();

    void step(double dt, int32 velocity_iterations, int32 position_iterations);

    b3Shape* create_shape(const std::filesystem::path& file_path);

    inline int get_shape_count() const {
        return m_shape_count;
    }

    b3Shape* get_shape_list() const {
        return m_shape_list;
    }

    // b3Shape* get_shape(int id) const {

    //     if (id >= m_shape_count)
    //         return nullptr;

    //     return m_shape_list[id];
    // }

    void add_shape(b3Shape* shape);

    void set_gravity(const b3Vector3d& gravity) {
        m_gravity = gravity;
    }

    b3Vector3d gravity() {
        return m_gravity;
    }

    b3BroadPhase* get_broad_phase() {
        return m_contact_manager.get_broad_phase();
    }

    /**
     * @brief Clear all objects in the world.
     */
    void clear();

    inline void awake_contact_check() {
        m_new_contacts = true;
    }

    void generate_island();

    inline b3BlockAllocator* get_block_allocator() {
        return &m_block_allocator;
    }

protected:

    void solve(double delta_t);


};


#endif //BOX3D_B3_WORLD_HPP
