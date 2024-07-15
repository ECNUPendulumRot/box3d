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


#ifndef BOX3D_B3PLANESHAPE_HPP
#define BOX3D_B3PLANESHAPE_HPP


#include "geometry/b3_shape.hpp"

/**
 * @brief represents a plane shape in a 3D physics engine.
 */
class b3PlaneShape : public b3Shape {

public:

    /**
     * @brief m_half_length: Represents half of the plane's length.
     * m_half_width: Represents half of the plane's width.
     */
    real m_half_length, m_half_width;

    /**
     * @brief Stores the four vertices of the plane in 3D space.
     */
    b3Vec3r m_vertices[4];

    /**
     * @brief the constructor of b3PlaneShape
     */
    b3PlaneShape();

    /**
     * @brief Defines the plane's dimensions and calculates its vertices.
     * @param point: a point on this plane
     * @param normal: the normal of this plane
    */
    void set_as_plane(real length, real width);

    /**
     * @brief Returns the number of child shapes. For a plane, this is always 1.
     * @return
     */
    int32 get_child_count() const {
        return 1;
    }

    /**
     * @brief Computes the axis-aligned bounding box (AABB) for the plane.
     * @param aabb Pointer to the AABB to be computed.
     * @param xf Transformation applied to the plane.
     * @param child_index Index of the child shape.
     */
    void get_bound_aabb(b3AABB* aabb, const b3Transr& xf, int32 child_index) const override;

    /**
     * @brief Calculates the mass properties of the plane. Since a plane is a static object,
     * its mass and inertia are zero.
     * @param mass_data Reference to the mass properties to be computed.
     * @param density Density of the plane.
     */
    void compute_mass_properties(b3MassProperty& mass_data, real density) const override;

    /**
     * @brief Creates and returns a deep copy of the current b3PlaneShape object.
     * @return A pointer to the newly created clone of the b3PlaneShape object.
     */
    b3Shape* clone() const override;
};


#endif  // BOX3D_B3PLANESHAPE_HPP