
#ifndef BOX3D_B3_ALLOCATOR_HPP
#define BOX3D_B3_ALLOCATOR_HPP


#include <malloc.h>
#include "common/b3_types.hpp"


inline void* b3_alloc(int32 size) {
    return malloc(size);
}

inline void b3_free(void* mem) {
    free(mem);
}


#endif //BOX3D_B3_ALLOCATOR_HPP
