
#ifndef BOX3D_B3_MESH_HPP
#define BOX3D_B3_MESH_HPP


#include <string>
#include <vector>
#include <filesystem>

#include "common/b3_types.hpp"

#include "geometry/b3_shape.hpp"


class b3Mesh: public b3Shape {

    friend class b3World;

    /**
     * @brief Vertices of the mesh
     * @details Each row is vertex's x, y, z coordinates
     */
    E3MatrixXd m_V;

    /**
     * @brief Edges of the mesh
     * @details Each row is an edge with 2 indexes to vertex
     */
    E3MatrixXi m_E;

    /**
     * @brief Faces of the mesh
     * @details Each row is a face with 3 indexes to vertex
     */
    E3MatrixXi m_F;

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


    void get_bound_aabb(b3AABB* aabb, const b3TransformD& xf, int32 childIndex) const override;

    int32 get_child_count() const override {
        return 1;
    }

    // TODO: check this method. the density is passed
    void compute_mass_properties(b3MassProperty& mass_data, double density) const override;

    b3Shape* clone() const override;

    void init_view_data() override;

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
    void recenter(const b3TransformD& new_center);

    E3MatrixXd transform() const;

    E3MatrixXd transform_rigid(const b3TransformD& pose) const;

    /**
     * @brief Get volume, center of geometry and inertia from the mesh
     * @param volume: volume of the mesh
     * @param CoG: center of geometry of the mesh
     * @param Inertia: inertia of the mesh
     * @return true on success, false on errors
     */
    bool mesh_properties(double& volume, b3Vector3d& CoG, E3Matrix3d& Inertia) const;

    /**
     * @brief Get the vertices of the mesh
     */
    inline E3MatrixXd& vertices() {
        return m_V;
    }

    /**
     * @brief Get the edges of the mesh
     */
    inline E3MatrixXi& edges() {
        return m_E;
    }

    // TODO: delete this method
    void test_addition() {
        m_V.rowwise() += Eigen::RowVector3d(1, 1, 1);
    }

    /**
     * @brief Get the faces of the mesh
     */
    inline E3MatrixXi& faces() {
        return m_F;
    }

    inline void set_next(b3Mesh* next) {
        m_next = next;
    }

    /**
     * @brief get the support vector
     * @param wd: the vector is in the world frame
    */
    // Eigen::Vector3d get_support(const Eigen::Vector3d& wd);


};


#endif //BOX3D_B3_MESH_HPP
