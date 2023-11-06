
#ifndef BOX3D_B3_BODY_HPP
#define BOX3D_B3_BODY_HPP


#include "geometry/b3_mesh.hpp"
#include "dynamics/b3_pose.hpp"

namespace box3d {

    class b3Body;

}


class box3d::b3Body {

    /**
     * @brief Mesh of the body
     */
    b3Mesh* m_mesh;

public:

    /**
     * @brief Construct a new b3Body object
     */
    b3Body();

    /**
     * @brief Construct a new b3Body object
     * @param obj_file_name: The obj file name
     */
    explicit b3Body(const std::string& obj_file_name);

    inline void set_mesh(const std::string& obj_file_name) {
        m_mesh = new b3Mesh(obj_file_name);
    };

    inline b3Mesh* mesh() const {
        return m_mesh;
    }
};



#endif //BOX3D_B3_BODY_HPP
