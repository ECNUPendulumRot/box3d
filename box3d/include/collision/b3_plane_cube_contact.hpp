
#ifndef BOX3D_B3_PLANE_CUBE_CONTACT_HPP
#define BOX3D_B3_PLANE_CUBE_CONTACT_HPP


#include "collision/b3_contact.hpp"

/**
 * @brief The b3PlaneCubeContact class represents a specific type of contact
 * between a plane and a cube in a physics simulation.
 */
class b3PlaneCubeContact: public b3Contact {

public:

    /**
     * @brief Static method to create a b3PlaneCubeContact object given two fixtures and a block allocator
     * @param fixture_a Pointer to the first fixture (plane).
     * @param index_a Index related to fixture A
     * @param fixture_b Pointer to the second fixture (cube).
     * @param index_b Index related to fixture B
     * @param block_allocator Pointer to the block allocator for memory management.
     * @return Returns a pointer to the newly created b3PlaneCubeContact object.
     */
    static b3Contact* create(b3Fixture* fixture_a, int32 index_a, b3Fixture* fixture_b, int32 index_b, b3BlockAllocator* block_allocator);

    /**
     * @brief Static method to destroy a b3PlaneCubeContact object and free its memory using a block allocator.
     * @param contact Pointer to the contact object to destroy.
     * @param block_allocator Pointer to the block allocator for memory management.
     */
    static void destroy(b3Contact* contact, b3BlockAllocator* block_allocator);

    /**
     * @brief  Constructor that initializes the contact with two fixtures (a plane and a cube).
     * @param fixture_a Pointer to the first fixture (plane).
     * @param fixture_b Pointer to the second fixture (cube).
     */
    b3PlaneCubeContact(b3Fixture* fixture_a, b3Fixture* fixture_b);

    /**
     * @brief Virtual destructor to properly destroy instances of b3PlaneCubeContact.
     */
    virtual ~b3PlaneCubeContact() = default;

    /**
     * @brief Method to evaluate and compute the collision manifold (contact points) between the plane and cube.
     * @param manifold Pointer to the manifold structure where the collision information is stored.
     * @param xf_a Transform of the first fixture (plane).
     * @param xf_b Transform of the second fixture (cube).
     */
    void evaluate(b3Manifold* manifold, const b3Transr& xf_a, const b3Transr& xf_b) override;

};


#endif //BOX3D_B3_PLANE_CUBE_CONTACT_HPP
