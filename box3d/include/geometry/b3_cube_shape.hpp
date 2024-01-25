
#ifndef BOX3D_B3_CUBE_SHAPE_HPP
#define BOX3D_B3_CUBE_SHAPE_HPP


#include "geometry/b3_shape.hpp"


struct b3EdgeIndex {

    int32 v1;

    int32 v2;

};


union b3FaceIndex {

    int32 e[4];

    struct {
        int32 e1;

        int32 e2;

        int32 e3;

        int32 e4;
    };

};


class b3CubeShape : public b3Shape {

public:

    b3Vector3d m_centroid;

    b3Vector3d m_vertices[8];

    b3Vector3d m_normals[6];

    b3EdgeIndex m_edges[12];

    b3FaceIndex m_faces[6];

    b3Vector3d m_xyz;

    b3Vector3d m_h_xyz;

    b3CubeShape();

    /**
     * @brief create the cube with length, width and height
     * @param hx: the half width of the cube
     * @param hy: the half length of the cube
     * @param hz: the half height of the cube
     */
    void set_as_box(double hx, double hy, double hz);

    virtual ~b3CubeShape() = default;

    int32 get_child_count() const override {
        return 1;
    }


    void get_bound_aabb(b3AABB* aabb, const b3TransformD& xf, int32 childIndex) const override;

    void compute_mass_properties(b3MassProperty& mass_data, double density) const override;

    void init_view_data() override;

    void setup_view_data(const b3TransformD& xf) override;

    b3Shape* clone() const override;

};


#endif //BOX3D_B3_CUBE_SHAPE_HPP