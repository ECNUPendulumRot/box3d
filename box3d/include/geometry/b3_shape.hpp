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


#ifndef BOX3D_B3_SHAPE_HPP
#define BOX3D_B3_SHAPE_HPP


#include "common/b3_types.hpp"
#include "dynamics/b3_transform.hpp"


/////////// Forward Declaration ///////////

class b3Body;

class b3AABB;

class b3MassProperty;

class b3BlockAllocator;

//////////////////////////////////////////

/**
 * @brief  define the types of shapes available in a physics engine
 */
enum b3ShapeType {
    e_type_not_defined = -1, ///< Represents an undefined shape type.
    e_cube       = 0, ///< Represents a cube shape.
    e_sphere     = 1, ///< Represents a sphere shape.
    e_plane      = 2, ///< Represents a plane shape.
    e_type_count = 3  ///< Represents the count of defined shape types.
};

/**
 * @brief The b3Shape class serves as a base class for different types of shapes in
 * a physics engine.
 */
class b3Shape {

    /**
     * @brief b3DistanceProxy is the friend class of b3Shape class
     */
    friend class b3DistanceProxy;

protected:

    /**
     * @brief Specifies the type of the shape
     */
    b3ShapeType m_type = e_type_not_defined;

    /**
     * @brief The radius of the shape.
     * This is used for extending the AABB of the shape for collision test
     */
    real m_radius = 0;

    /**
     * @brief Next mesh in the list
     * This is used for managing memory of meshes.
     */
    b3Shape* m_next = nullptr;

    /**
     * @brief The body that the mesh belongs to.
     */
    b3Body* m_body = nullptr;

    /**
     * @brief Used for memory allocation.
     */
    b3BlockAllocator* m_block_allocator = nullptr;

    /**
     * @brief This is used for gui
     */
    b3Vec3r m_color = b3Vec3r::zero();

public:
    b3Vec3r m_centroid;
    /**
     * @brief Destructor for the b3Shape class.
     */
    virtual ~b3Shape() = default;

    /**
     * @brief Returns the number of child shapes.
     * @return
     */
    virtual int32 get_child_count() const {
        return 0;
    };

    /**
     * @brief Calculates the Axis-Aligned Bounding Box (AABB) for the shape.
     * @param aabb A pointer to an AABB object that will store the calculated bounding box.
     * @param xf A transformation object representing the shape's position and orientation.
     * @param child_index The index of the child shape
     */
    virtual void get_bound_aabb(b3AABB* aabb, const b3Transr& xf, int32 child_index) const {
        b3_NOT_USED(aabb);
        b3_NOT_USED(xf);
        b3_NOT_USED(child_index);
    };

    /**
     * @brief Computes the mass properties of the shape
     * @param mass_data A reference to a b3MassProperty object that will store the
     * calculated mass properties.
     * @param density The density of the material.
     */
    virtual void compute_mass_properties(b3MassProperty& mass_data, real density) const {
        b3_NOT_USED(mass_data);
        b3_NOT_USED(density);
    };

    /**
     * @brief Creates a copy of the shape.
     * @return returns nullptr, but derived classes can override this to return a new
     * instance of the shape.
     */
    virtual b3Shape* clone() const {
        return nullptr;
    };

    /**
     * @brief gets the type of the shape.
     * @return Returns the type of the shape.
     */
    b3ShapeType get_type() const {
        return m_type;
    }

    /**
     * @brief Sets the body to which the shape belongs.
     * @param body A pointer to the body.
     */
    void set_relative_body(b3Body* body) {
        m_body = body;
    }

    /**
     * @brief Sets the block allocator for memory management.
     * @param block_allocator A pointer to the block allocator.
     */
    void set_block_allocator(b3BlockAllocator* block_allocator) {
        m_block_allocator = block_allocator;
    }

    /**
     * @brief gets a pointer to the next shape in the linked list.
     * @return Returns a pointer to the next shape in the linked list.
     */
    b3Shape* next() const {
        return m_next;
    }

    /**
     * @brief Sets the next shape in the linked list.
     * @param next A pointer to the next shape.
     */
    void set_next(b3Shape* next) {
        m_next = next;
    }

    /**
     * @brief gets the radius of the shape.
     * @return Returns the radius of the shape.
     */
    virtual real get_radius() const {
        return m_radius;
    }

    /**
     * @brief gets the body to which the shape belongs.
     * @return Returns the body to which the shape belongs.
     */
    b3Body* get_body() const {
        return m_body;
    }

    /**
     * @brief Sets the color of the shape.
     * @param color A reference to a vector representing the color.
     */
    void set_color(const b3Vec3r& color) {
        m_color = color;
    }

    /**
     * @brief gets the color of the shape.
     * @return Returns the color of the shape.
     */
    b3Vec3r get_color() const {
        return m_color;
    }
};


#endif //BOX3D_B3_SHAPE_HPP
