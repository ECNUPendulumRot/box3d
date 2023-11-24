
#include "b3_distance_proxy.hpp"

int32 box3d::b3DistanceProxy::get_support(const b3Vector3d& d) const {

}

inline int32 box3d::b3DistanceProxy::get_vertex_count() const{
    return m_vertexs.rows();
}

b3Vector3d box3d::b3DistanceProxy::get_vertex(int32 index) const {
    auto item = m_vertexs.row(index);
    // return b3Vector3d(item);
    return b3Vector3d(item.x(), item.y(), item.z());
}