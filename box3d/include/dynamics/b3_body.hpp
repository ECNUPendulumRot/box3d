
#ifndef BOX3D_B3_BODY_HPP
#define BOX3D_B3_BODY_HPP

#include "geometry/b3_mesh.hpp"
#include "dynamics/b3_pose.hpp"

namespace box3d {

    template<typename T>
    class b3Body;

    class b3BodyDef;

    enum class b3BodyType;

}

template<typename T>
class box3d::b3Body {

    /**
     * @brief Mesh of the body
     */
    b3Mesh* m_mesh;

    T* m_next;

public:

    /**
     * @brief Construct a new b3Body object
     */
    b3Body():m_mesh(nullptr){
        ;
    };

    /**
     * @brief Construct a new b3Body object
     * @param obj_file_name: The obj file name
     */
    explicit b3Body(const std::string& obj_file_name){
        m_mesh = new b3Mesh(obj_file_name);
    }

    virtual void set_mesh(b3Mesh* mesh) {
        m_mesh = mesh;
    };

    inline b3Mesh* mesh() const {
        return m_mesh;
    }

    inline void set_next(T* next) {
        m_next = next;
    }

    inline T* next() const {
        return m_next;
    }

};


#endif //BOX3D_B3_BODY_HPP
