
#include "dynamics/b3_body.hpp"


box3d::b3Body::b3Body():
    m_mesh(nullptr)
{
    ;
}


box3d::b3Body::b3Body(const std::string &obj_file_name)
{
    m_mesh = new b3Mesh(obj_file_name);
}