
#ifndef BOX3D_B3_WORLD_HPP
#define BOX3D_B3_WORLD_HPP

#include <filesystem>

#include "dynamics/b3_body.hpp"
#include "dynamics/b3_body_def.hpp"

#include "geometry/b3_shape.hpp"

#include "collision/b3_broad_phase.hpp"
#include "collision/b3_contact_manager.hpp"

#include "dynamics/b3_island.hpp"

#include "common/b3_block_allocator.hpp"

#include "collision/b3_dispatcher.hpp"

struct b3Color;
struct b3TimeStep;
class b3Draw;

class b3World {

    /**
     * This is the all bodies in the world.
     * doubly linked list
     */
    b3Body* m_body_list;

    /**
     * This is the all shapes in the world.
     * doubly linked list
     */
    b3Shape* m_shape_list;

    int32 m_shape_count;

    int32 m_body_count;

    b3Vec3r m_gravity = b3Vec3r(0, 0, 0);

    real m_hz = 60;

    b3BlockAllocator m_block_allocator;

    b3ContactManager m_contact_manager;

    bool m_new_contacts = false;

    b3Draw* m_debug_draw;

    b3Dispatcher* m_dispatcher;

    b3DispatcherInfo m_dispatcher_info;

public:

    b3World();

    explicit b3World(const b3Vec3r& gravity);

    ~b3World();

    b3Body* create_body(const b3BodyDef& def);

    inline bool empty() const {
        return m_body_count == 0;
    }

    /**
     * This is simulation world time forward.
     * @param dt
     * @param velocity_iterations the numbers of iterations when solving velocity constraints.
     * @param position_iterations the numbers of iterations when solving position constraints. (but we have not use it)
     */
    void step(real dt, int32 velocity_iterations, int32 position_iterations);

    inline int get_shape_count() const {
        return m_shape_count;
    }

    b3Shape* get_shape_list() const {
        return m_shape_list;
    }

    void add_shape(b3Shape* shape);

    void set_gravity(const b3Vec3r& gravity) {
        m_gravity = gravity;
    }

    b3Vec3r get_gravity() const {
        return m_gravity;
    }

    b3BroadPhase* get_broad_phase() {
        return m_contact_manager.get_broad_phase();
    }

    b3Contact* get_contact_list() {
        return m_contact_manager.get_contact_list();
    }

    /**
     * @brief Clear all objects in the world.
     */
    void clear();

    inline void awake_contact_check() {
        m_new_contacts = true;
    }

    inline b3BlockAllocator* get_block_allocator() {
        return &m_block_allocator;
    }

    inline int32 get_body_count() const {
        return m_body_count;
    }

    inline b3Body* get_body_list() {
        return m_body_list;
    }

    inline void set_debug_draw(b3Draw* draw) {
        m_debug_draw = draw;

        m_dispatcher_info.m_debug_draw = m_debug_draw;
    }

    void debug_draw();

private:

    // void solve(double delta_t);

    void solve(b3TimeStep& step);

public:

    void draw_shape(b3Fixture* fixture, const b3Transformr& xf, const b3Color& color);
};


#endif //BOX3D_B3_WORLD_HPP
