// The MIT License

// Copyright (c) 2024
// Robot Motion and Vision Laboratory at East China Normal University
// Contact: tophill.robotics@gmail.com

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


#ifndef BOX3D_B3_CUBE_SHAPE_HPP
#define BOX3D_B3_CUBE_SHAPE_HPP


#include "geometry/b3_shape.hpp"

/**
 * @brief  represent an edge in a mesh or graph by storing the indices of the two
 * vertices that form the edge.
 */
struct b3EdgeIndex {

    /**
     * @brief Represents the index of the first vertex of the edge.
     */
    int32 v1;

    /**
     * @brief Represents the index of the second vertex of the edge.
     */
    int32 v2;

};

/**
 * @brief represent a face in a mesh by storing the indices of the edges that form the face.
 */
struct b3FaceIndex {

    /**
     * @brief Stores the indices of the edges that form the face.
     */
    union {
        int32 e[4]; ///< Array of edge indices.

        struct {
            int32 e1; ///< Individual edge indices.

            int32 e2; ///< Individual edge indices.

            int32 e3; ///< Individual edge indices.

            int32 e4; ///< Individual edge indices.
        };
    };

};

/**
 * @brief represent a 3D cube shape in a physics engine
 */
class b3CubeShape : public b3Shape {

public:

    /**
     * @brief Stores the centroid (geometric center) of the cube.
     */
    //b3Vec3r m_centroid;

    /**
     * @brief Array of the cube's vertices.
     */
    b3Vec3r m_vertices[8];

    /**
     * @brief Array of the cube's face normals.
     */
    b3Vec3r m_normals[6];

    /**
     * @brief Array of the cube's edges.
     */
    b3EdgeIndex m_edges[12];

    /**
     * @brief  Array of the cube's faces.
     */
    b3FaceIndex m_faces[6];

    /**
     * @brief Stores the dimensions of the cube
     */
    b3Vec3r m_xyz;

    /**
     * @brief Stores half the dimensions of the cube
     */
    b3Vec3r m_h_xyz;

    /**
     * @brief the constructor of b3CubeShape
     */
    b3CubeShape();

    /**
     * @brief create the cube with length, width and height
     * @param hx: the half width of the cube
     * @param hy: the half length of the cube
     * @param hz: the half height of the cube
     */
    void set_as_box(double hx, double hy, double hz);
    void set_as_box(double hx, double hy, double hz, b3Vec3r center, b3Vec3r angle);
    /**
     * @brief the destructor  of b3CubeShape
     */
    virtual ~b3CubeShape() = default;

    /**
     * @brief gets the number of child shapes.
     * @return Returns the number of child shapes.
     */
    int32 get_child_count() const override {
        return 1;
    }

    /**
     * @brief compute the axis-aligned bounding box (AABB) for the cube shape
     * @param aabb A pointer to a b3AABB object where the resulting AABB will be stored.
     * @param xf A constant reference to a b3Transr object representing the transformation to apply to the cube.
     * @param childIndex An integer representing the index of the child shape. Not used
     * in this function as a cube is considered a single shape.
     */
    void get_bound_aabb(b3AABB* aabb, const b3Transr& xf, int32 childIndex) const override;

    /**
     * @brief calculate and set the mass properties of the cube shape, given its density.
     * @param mass_data A reference to a b3MassProperty object where the calculated mass
     * properties will be stored.
     * @param density A real number representing the density of the material of the cube.
     */
    void compute_mass_properties(b3MassProperty& mass_data, real density) const override;

    /**
     * @brief create and return a deep copy of the current b3CubeShape object.
     * @return return a deep copy of the current b3CubeShape object.
     */
    b3Shape* clone() const override;

};


#endif //BOX3D_B3_CUBE_SHAPE_HPP
