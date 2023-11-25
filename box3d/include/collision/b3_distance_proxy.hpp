
#ifndef B3_DISTANCE_PROXY_HPP
#define B3_DISTANCE_PROXY_HPP

#include "common/b3_types.hpp"


namespace box3d {
    class b3Body;
    class b3Fixture;

    class b3DistanceProxy;
}

class box3d::b3DistanceProxy {
    // const b3MatrixXd* m_vertices;
    //  TODO: convert to pointer
    const b3MatrixXd m_vertexs;

    // TODO: store collision pointS
    b3Vector3d m_buffer[3];
    int32 m_count = 0;
public:
    b3DistanceProxy(const b3Body* body) : 
            m_vertexs(body->mesh()->transform()) {}

    b3DistanceProxy(const b3Fixture* fixture) : 
            m_vertexs(fixture->get_mesh()->transform()) {}

    /**
     * @description: 给定方向d，寻找在d上的投影最大值
     * @param {b3Vector3d&} d 3*1
     * @return {*}
     */    
    b3Vector3d get_support(const b3Vector3d& d) const;

    inline int32 get_vertex_count() const;

    b3Vector3d get_vertex(int32 index) const; 
};

#endif