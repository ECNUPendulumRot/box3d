// The MIT License

// Copyright (c) 2024
// Robot Motion and Vision Laboratory at East China Normal University
// Contact: tophill.robotics@gmail.com

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


#ifndef BOX3D_B3_BLOCK_ALLOCATOR_HPP
#define BOX3D_B3_BLOCK_ALLOCATOR_HPP


#include <memory>

#include "common/b3_types.hpp"


/////////// Forward Declaration ///////////

struct b3Block;
struct b3Chunk;

//////////////////////////////////////////


const int32 b3_block_size_count = 14; ///< size type of blocks. There are 14 types of block


/**
 * @brief This is a small object allocator used for allocating small
 * objects that persist for more than one time step.
 * @see http://www.codeproject.com/useritems/Small_Block_Allocator.asp
 */
class b3BlockAllocator {

    /**
     * @brief A large block of memory for allocating blocks.
     * The blocks in this chunk are fixed size, aligned and in the form of a list.
     */
    b3Chunk* m_chunks;

    /**
     * @brief current count of max allocated chunks.
     */
    int32 m_chunk_space;

    /**
     * @brief the number of chunks used from.
     * This is smaller or equal to m_chunk_space.
     */
    int32 m_chunk_count;

    /**
     * @brief b3_block_size_count block slots point to the last usable block of certain size.
     */
    b3Block* m_free_lists[b3_block_size_count];

public:

    /**
     * @brief constructor of b3BlockAllocator
     */
    b3BlockAllocator();

    /**
     * @brief destructor of b3BlockAllocator
     */
    ~b3BlockAllocator();

    /**
     * @brief allocate a block of memory with size.
     * @param size size of the block in bytes.
     * @return a pointer to the allocated block.
     *     @retval nullptr if size is 0 or fail to allocate.
     * @attention Blocks larger than b3_max_block_size will be directly allocated from heap.
     * This will affect the performance of the allocator.
     * Check and change the block size map in .cpp if needed.
     */
    void* allocate(int32 size);

    /**
     * @brief free a block of memory.
     * @param p pointer to the block.
     * @param size size of the block in bytes.
     * @attention Blocks larger than b3_max_block_size will free from heap.
     *
     */
    void free(void* p, int32 size);

    /**
     * @brief clear all allocated blocks.
     */
    void clear();

};


#endif // BOX3D_B3_BLOCK_ALLOCATOR_HPP