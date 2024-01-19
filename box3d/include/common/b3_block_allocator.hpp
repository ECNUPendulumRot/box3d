
#ifndef BOX3D_B3_BLOCK_ALLOCATOR_HPP
#define BOX3D_B3_BLOCK_ALLOCATOR_HPP

#include "common/b3_types.hpp"

const int32 b3_block_size_count = 14;

struct b3Block;

struct b3Chunk;


/// This is a small object allocator used for allocating small
/// objects that persist for more than one time step.
/// See: http://www.codeproject.com/useritems/Small_Block_Allocator.asp

class b3BlockAllocator {

    b3Chunk* m_chunks;

    int32 m_chunk_count;

    int32 m_chunk_space;

    b3Block* m_free_lists[b3_block_size_count];

public:

    b3BlockAllocator();

    ~b3BlockAllocator();

    void* allocate(int32 size);

    void free(void* p, int32 size);

    void clear();

};

#endif // BOX3D_B3_BLOCK_ALLOCATOR_HPP