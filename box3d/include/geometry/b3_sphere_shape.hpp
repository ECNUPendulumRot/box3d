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


#ifndef BOX3D_B3SPHERESHAPE_HPP
#define BOX3D_B3SPHERESHAPE_HPP


#include "geometry/b3_shape.hpp"

/**
 * @brief represents a spherical shape in a physics engine.
 */
class b3SphereShape : public b3Shape {

    /**
     * @brief the friend class of b3SphereShape
     */
    friend class b3DistanceProxy;

    /**
     * @brief Represents the center point (centroid) of the sphere.
     */
    b3Vec3r m_centroid;

public:

    /**
     * @brief constructor of b3SphereShape
     */
    b3SphereShape();

    /**
     * @brief gets the number of child shapes. For a sphere, this is always 1.
     * @return Returns the number of child shapes. For a sphere, this is always 1.
     */
    int32 get_child_count() const override {
        return 1;
    }

    /**
     * @brief gets the centroid (center point) of the sphere.
     * @return Returns the centroid (center point) of the sphere.
     */
    b3Vec3r get_centroid() const {
        return m_centroid;
    }

    /**
     * @brief when create a sphere, we need call this function at first.
     * @param radius
     */
    void set_as_sphere(real radius);

    /**
     * @brief Creates a copy of the b3SphereShape object.
     * @return A pointer to the new b3SphereShape instance.
     */
    b3Shape* clone() const override;

    /**
     * @brief Calculates the Axis-Aligned Bounding Box (AABB) for the sphere.
     * @param aabb A pointer to an AABB object that will store the calculated bounding box.
     * @param xf A transformation object representing the sphere's position and orientation.
     * @param child_index The index of the child shape
     */
    void get_bound_aabb(b3AABB* aabb, const b3Transr& xf, int32 child_index) const override;

    /**
     * @brief Computes the mass properties of the sphere
     * @param mass_data A reference to a b3MassProperty object that will store the
     * calculated mass properties.
     * @param density The density of the material.
     */
    void compute_mass_properties(b3MassProperty& mass_data, real density) const override;

    /**
     * @brief gets the radius of the sphere.
     * @return Returns the radius of the sphere.
     */
    real get_radius() const override {
        return m_radius;
    }
};


#endif // BOX3D_B3SPHERESHAPE_HPP