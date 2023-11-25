
#include "b3_distance_proxy.hpp"

b3Vector3d box3d::b3DistanceProxy::get_support(const b3Vector3d& d) const {
    
    int max_index = 0;
    double max_value = get_vertex(0).dot(d);

    for(int i = 1; i < get_vertex_count(); ++i) {
        double value = get_vertex(i).dot(d);
        if(value > max_value) {
            max_value = value;
            max_index = i;
        }
    }
    return get_vertex(max_index);
}

inline int32 box3d::b3DistanceProxy::get_vertex_count() const{
    return m_vertexs.rows();
}

b3Vector3d box3d::b3DistanceProxy::get_vertex(int32 index) const {
    auto item = m_vertexs.row(index);
    // return b3Vector3d(item);
    return b3Vector3d(item.x(), item.y(), item.z());
}