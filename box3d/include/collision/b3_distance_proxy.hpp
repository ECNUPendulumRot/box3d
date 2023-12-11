
#ifndef B3_DISTANCE_PROXY_HPP
#define B3_DISTANCE_PROXY_HPP

#include "common/b3_types.hpp"
#include "dynamics/b3_body.hpp"
#include "collision/b3_fixture.hpp"
#include "geometry/b3_mesh.hpp"

#include "utils/b3_log.hpp"

namespace box3d {
    class b3Body;
    class b3Fixture;

    class b3DistanceProxy;
}

class box3d::b3DistanceProxy
{

    // const b3MatrixXd* m_vertices;
    //  TODO: convert to pointer
    // b3MatrixXd m_vertexs;
    b3Mesh* m_mesh;
    // TODO: store collision pointS

    b3Vector3d m_buffer[3];

    int32 m_count = 0;

public:

    b3DistanceProxy(const b3Body* body): 
            m_mesh(body->mesh()) {}

    b3DistanceProxy(const b3Fixture* fixture): 
            m_mesh(fixture->get_mesh()) {}

    /**
     * @description: 给定方向d，寻找在d上的投影最大值
     * @param {Eigen::Vector3d} d 3*1
     * @return {*}
     */    
    Eigen::Vector3d get_support(Eigen::Vector3d d) const {
        
        return m_mesh->get_support(d);
        
        // auto res = m_mesh->get_support(d);
        // spdlog::info("support point is {}, {}, {}", res.x(), res.y(), res.z());
        // return res;
    }
};

#endif