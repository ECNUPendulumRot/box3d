
#include "box3d.hpp"

#include "collision/b3_contact.hpp"

#include "dynamics/b3_mass_property.hpp"

#include <iostream>

int main() {
    std::cout << "b3AABB: "             << sizeof(b3AABB)               << "  ";
    std::cout << "b3BroadPhase: "       << sizeof(b3BroadPhase)         << "  ";
    std::cout << "b3Body: "             << sizeof(b3Body)               << "  ";
    std::cout << "b3BodyType: "         << sizeof(b3BodyType)           << "  ";
    std::cout << "b3BodyDef: "          << sizeof(b3BodyDef)            << "  ";
    std::cout << "b3ContactEdge: "      << sizeof(b3ContactEdge)        << "  ";
    std::cout << "b3Contact: "          << sizeof(b3Contact)            << "  ";
    std::cout << "b3ClipVertex: "       << sizeof(b3ClipVertex)         << "  ";
    std::cout << "b3ContactFeature: "   << sizeof(b3ContactFeature)     << "  ";
    std::cout << "b3ContactID: "        << sizeof(b3ContactID)          << "  ";
    std::cout << "b3ContactManager: "   << sizeof(b3ContactManager)     << "  ";
    std::cout << "b3ContactRegister: "  << sizeof(b3ContactRegister)    << "  ";
    std::cout << "b3CubeShape: "        << sizeof(b3CubeShape)          << "  ";
    std::cout << "b3DynamicTree: "      << sizeof(b3DynamicTree)        << "  ";
    std::cout << "b3EdgeIndex: "        << sizeof(b3EdgeIndex)          << "  ";
    std::cout << "b3Fixture: "          << sizeof(b3Fixture)            << "  ";
    std::cout << "b3FaceIndex: "        << sizeof(b3FaceIndex)          << "  ";
    std::cout << "b3FixtureDef: "       << sizeof(b3FixtureDef)         << "  ";
    std::cout << "b3FixtureProxy: "     << sizeof(b3FixtureProxy)       << "  ";
    std::cout << "b3Island: "           << sizeof(b3Island)             << "  ";
    std::cout << "b3Manifold: "         << sizeof(b3Manifold)           << "  ";
    std::cout << "b3ManifoldPoint: "    << sizeof(b3ManifoldPoint)      << "  ";
    std::cout << "b3MassProperty: "     << sizeof(b3MassProperty)       << "  ";
    std::cout << "b3Matrix3f: "         << sizeof(b3Matrix3f)           << "  ";
    std::cout << "b3Matrix3r: " << sizeof(b3Matrix3r) << "  ";
    std::cout << "b3Node: "             << sizeof(b3Node)               << "  ";
    std::cout << "b3Pair: "             << sizeof(b3Pair)               << "  ";
    std::cout << "b3Shape: "            << sizeof(b3Shape)              << "  ";
    std::cout << "b3ShapeType: "        << sizeof(b3ShapeType)          << "  ";
    std::cout << "b3ShapeConfig: "      << sizeof(b3SphereConfig)       << "  ";
    std::cout << "b3SphereShape: "      << sizeof(b3SphereShape)        << "  ";
    std::cout << "b3Transr: " << sizeof(b3Transr) << "  ";
    std::cout << "b3Transformf: " << sizeof(b3Transformf) << "  ";
    std::cout << "b3ViewData: "         << sizeof(b3ViewData)           << "  ";
    std::cout << "b3Vec3f: "         << sizeof(b3Vec3f)           << "  ";
    std::cout << "b3VertexD: " << sizeof(b3Vec3r) << "  ";
    std::cout << "b3World: "            << sizeof(b3World)              << "  ";

    std::cout << std::endl;
}