
#ifndef BOX3D_B3_WORLD_HPP
#define BOX3D_B3_WORLD_HPP

#include  <filesystem>

#include "dynamics/b3_body.hpp"
#include "dynamics/b3_body_def.hpp"
#include "dynamics/b3_body_rigid.hpp"

#include "geometry/b3_mesh.hpp"

#include "collision/b3_broad_phase.hpp"


namespace box3d {

    class b3World;

    ////////////////

    class b3Body;

    class b3Mesh;

    class b3SolverAffine;
}


class box3d::b3World {

    friend class b3SolverAffine;

    b3Body* m_rigid_body_list;

    b3Body* m_affine_body_list;

    b3Mesh* m_mesh_list;

    int32 m_mesh_count;

    int32 m_rigid_body_count;

    int32 m_affine_body_count;

    b3Vector3d m_gravity = b3Vector3d(0, 0, 0);

    double m_hz = 60;

    b3BroadPhase m_broad_phase;

    b3SolverAffine* m_solver_affine;

public:

    b3World();

    ~b3World();

    // TODO: implement this
    b3Body* create_body(const b3BodyDef& def);

    inline bool empty() const {
        bool e = (m_rigid_body_count  == 0);
        e = e && (m_affine_body_count == 0);

        return e;
    }

    void test_step();

    b3Body* create_rigid_body(const b3BodyDef& def);

    b3Body* create_affine_body(const b3BodyDef& def);

    b3Mesh* create_mesh(const std::filesystem::path& file_path);

    inline int get_mesh_count() const {
        return m_mesh_count;
    }

    b3Mesh* get_mesh(int id) const {

        if (id >= m_mesh_count)
            return nullptr;

        return &m_mesh_list[id];
    }

    void set_gravity(const b3Vector3d& gravity) {
        m_gravity = gravity;
    }

    b3Vector3d gravity() {
        return m_gravity;
    }

    /**
     * @brief Clear all objects in the world.
     */
    void clear();

    b3BroadPhase* get_broad_phase() {
        return &m_broad_phase;
    }

protected:

    void solve_rigid(double delta_t);

};


#endif //BOX3D_B3_WORLD_HPP
