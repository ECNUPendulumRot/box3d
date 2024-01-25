
#include "geometry/b3_shape.hpp"
#include "common/b3_block_allocator.hpp"


b3Shape::~b3Shape() {
    m_block_allocator->free(m_view_data.m_V, 3 * sizeof(double) * m_view_data.m_vertex_count);
    m_block_allocator->free(m_view_data.m_F, 3 * sizeof(int) * m_view_data.m_face_count);
}
