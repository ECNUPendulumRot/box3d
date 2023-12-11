
#ifndef BOX3D_B3_MESH_HPP
#define BOX3D_B3_MESH_HPP


#include <string>
#include <vector>
#include <filesystem>

#include "common/b3_types.hpp"
#include "dynamics/b3_inertia.hpp"
#include "dynamics/b3_pose.hpp"

#include "collision/b3_aabb.hpp"


namespace box3d {

    class b3Mesh;

///////////////////////////

    class b3World;

}

class box3d::b3Mesh {

    friend class b3World;

    /**
     * @brief Vertices of the mesh
     * @details Each row is vertex's x, y, z coordinates
     */
    b3MatrixXd m_V;

    /**
     * @brief Edges of the mesh
     * @details Each row is an edge with 2 indexes to vertex
     */
    b3MatrixXi m_E;

    /**
     * @brief Faces of the mesh
     * @details Each row is a face with 3 indexes to vertex
     */
    b3MatrixXi m_F;

    /**
     * @brief Pose of the mesh wrt the world frame
     */
    b3PoseD* m_rel_pose = nullptr;

    /**
     * @brief Next mesh in the list
     * This is used for managing memory of meshes.
     */
    b3Mesh* m_next = nullptr;

    static std::vector<b3Mesh*> s_meshes;

public:

    /**
     * @brief Construct a new b3Mesh object
     */
    b3Mesh();

    /**
     * @brief Construct a new b3Mesh object
     * @param obj_file_name: path to .obj file
     */
    explicit b3Mesh(const std::string& obj_file_name);

    /**
     * @brief Read a mesh from an ascii obj file
     * @param obj_file_name: path to .obj file
     * @return true on success, false on errors
     * In obj, the coordinate is different from the common ones,
     * In obj, it is a x right, y up, z forward coordinate,
     * so we need to transform to a x forward, y left, z up coordinate
     */
    bool read_obj(const std::string& obj_file_name);

    /**
     * @brief align the mesh vertices to a new center
     * @param new_center: new center of the mesh
     */
    void recenter(const b3PoseD& new_center);

    b3MatrixXd transform() const;

    b3MatrixXd transform(const b3PoseD* pose) const;

    void set_relative_pose(b3PoseD* pose) {
        m_rel_pose = pose;
    }

    /**
     * @brief Get volume, center of geometry and inertia from the mesh
     * @param volume: volume of the mesh
     * @param CoG: center of geometry of the mesh
     * @param Inertia: inertia of the mesh
     * @return true on success, false on errors
     */
    bool mesh_properties(double& volume, b3PoseD& CoG, b3Inertia& Inertia) const;

    b3AABB get_bounding_aabb() const;

    /**
     * @brief Get the vertices of the mesh
     */
    inline b3MatrixXd& vertices() {
        return m_V;
    }

    /**
     * @brief Get the edges of the mesh
     */
    inline b3MatrixXi& edges() {
        return m_E;
    }

    // TODO: delete this method
    void test_addition() {
        m_V.rowwise() += Eigen::RowVector3d(1, 1, 1);
    }

    /**
     * @brief Get the faces of the mesh
     */
    inline b3MatrixXi& faces() {
        return m_F;
    }

    inline void set_next(b3Mesh* next) {
        m_next = next;
    }

    inline b3Mesh* next() const {
        return m_next;
    }

    /**
     * @brief get the support vector
     * @param wd: the vector is in the world frame
    */
    Eigen::Vector3d get_support(const Eigen::Vector3d& wd);


};


#endif //BOX3D_B3_MESH_HPP
