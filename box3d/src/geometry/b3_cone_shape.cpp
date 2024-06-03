
#include "geometry/b3_cone_shape.hpp"


void b3ConeShape::set_as_cone(real radius, real height)
{
    m_radius = radius;
    m_height = height;
    m_sin_angle = m_radius / b3_sqrt(m_radius * m_radius + m_height * m_height);
}





