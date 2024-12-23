
#include "collision/b3_octree.hpp"

b3OCNode::b3OCNode(b3AABB* aabb, int depth) : m_depth(depth) {
    m_center = (aabb->max() + aabb->min()) * 0.5;
    m_extend = (aabb->max() - aabb->min()) * 0.5;
}


b3OCNode::~b3OCNode() {
    m_objects.clear();
}
