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

#include "common/b3_block_allocator.hpp"

#include <cstring>
#include <climits>

#include "common/b3_allocator.hpp"
#include "common/b3_common.hpp"


static constexpr int32 b3_chunk_size = 16 * 1024; ///< size of on chunk, default is 16KB

static constexpr int32 b3_max_block_size = 640; ///< maximun size of one block

static constexpr int32 b3_chunk_array_increment = 128; ///< size of chunks to increase if current chunks are not enough

/// size of each block. Other size of blocks will be rounded to the nearest larger one.
static const int32 b3_block_sizes[b3_block_size_count] =  {
    16,     // 0
    32,     // 1
    64,     // 2
    96,     // 3
    128,    // 4
    160,    // 5
    192,    // 6
    224,    // 7
    256,    // 8
    320,    // 9
    384,    // 10
    448,    // 11
    512,    // 12
    640     // 13
};


/**
 * @brief This map an arbitrary allocation size to a suitable slot in b3_block_sizes
 */
struct b3SizeMap {

    /**
     * @brief map of different block sizes to corresponding rounded blocks.
     * @example a size of 17 will be rounded to m_values[17] = 32.
     */
    uint8 m_values[b3_max_block_size + 1];

    /**
     * @brief constructor of b3SizeMap
     * This will construct the m_values mapping.
     */
    b3SizeMap() {

        int32 j = 0;
        m_values[0] = 0;

        for (int32 i = 1; i <= b3_max_block_size; ++i) {
            if (i > b3_block_sizes[j]) {
                ++j;
            }
            m_values[i] = (uint8)j;
        }
    }
};


static const b3SizeMap b3_size_map; ///< map of different block sizes to corresponding rounded blocks.

/**
 * @brief The b3Chunk struct is designed to represent a chunk of memory allocated for
 * storing b3Block objects or data blocks.
 */
struct b3Chunk {
    /**
     * @brief Pointer to an array or collection of b3Block objects or data.
     */
    b3Block *m_blocks;
};

/**
 * @brief The b3Block struct is designed as a node structure for building linked lists
 * or chains of blocks.
 */
struct b3Block {
    /**
     * @brief Pointer to the next b3Block in the sequence.
     */
    b3Block *m_next;
};

/**
 * @brief constructor of b3BlockAllocator
 */
b3BlockAllocator::b3BlockAllocator()
{
    b3_assert(b3_block_size_count < UCHAR_MAX);

    m_chunk_space = b3_chunk_array_increment;
    m_chunk_count = 0;
    m_chunks = (b3Chunk *)b3_alloc(m_chunk_space * sizeof(b3Chunk));

    memset(m_chunks, 0, m_chunk_space * sizeof(b3Chunk));
    memset(m_free_lists, 0, sizeof(m_free_lists));
}

/**
 * @brief destructor of b3BlockAllocator
 */
b3BlockAllocator::~b3BlockAllocator()
{
    for(int32 i = 0; i < m_chunk_count; ++i) {
        b3_free(m_chunks[i].m_blocks);
    }
    b3_free(m_chunks);
}

/**
 * @brief allocate a block of memory with size.
 * @param size size of the block in bytes.
 * @return a pointer to the allocated block.
 *     @retval nullptr if size is 0 or fail to allocate.
 * @attention Blocks larger than b3_max_block_size will be directly allocated from heap.
 * This will affect the performance of the allocator.
 * Check and change the block size map in .cpp if needed.
 */
void* b3BlockAllocator::allocate(int32 size)
{
    if(size == 0) {
        return nullptr;
    }

    b3_assert(size > 0);

    if (size > b3_max_block_size) {
  	    return b3_alloc(size);
    }

    int32 index = b3_size_map.m_values[size];
    b3_assert(index >= 0 && index < b3_block_size_count);

    if (m_free_lists[index]) {
        // get the first free block in the list
        b3Block *block = m_free_lists[index];
        m_free_lists[index] = block->m_next;

        return block;
    } else {
        // allocate a new chunk and construct the list of blocks
        // if current chunks are all full, expand current chunk size
        if (m_chunk_count == m_chunk_space) {
            b3Chunk *old_chunk = m_chunks;
            m_chunk_space += b3_chunk_array_increment;
            m_chunks = (b3Chunk *)b3_alloc(m_chunk_space * sizeof(b3Chunk));
            memcpy(m_chunks, old_chunk, m_chunk_count * sizeof(b3Chunk));
            memset(m_chunks + m_chunk_count, 0, b3_chunk_array_increment * sizeof(b3Chunk));
            b3_free(old_chunk);
        }

        b3Chunk *chunk = m_chunks + m_chunk_count;
        chunk->m_blocks = (b3Block *)b3_alloc(b3_chunk_size);
        int32 block_size = b3_block_sizes[index];
        int32 block_count = b3_chunk_size / block_size;

        b3_assert(block_count * block_size <= b3_chunk_size);

        // construct the block list
        int8 *start = (int8 *)chunk->m_blocks;
        for (int32 i = 0; i < block_count - 1; ++i) {
            b3Block *block = (b3Block *)(start);

            start += block_size;

            b3Block *next = (b3Block *)(start);
            block->m_next = next;
        }
        b3Block *last = (b3Block *)((int8 *)chunk->m_blocks + block_size * (block_count - 1));
        last->m_next = nullptr;

        m_free_lists[index] = chunk->m_blocks->m_next;
        ++m_chunk_count;

        return chunk->m_blocks;
    }
}

/**
 * @brief free a block of memory.
 * @param p pointer to the block.
 * @param size size of the block in bytes.
 * @attention Blocks larger than b3_max_block_size will free from heap.
 *
 */
void b3BlockAllocator::free(void* p, int32 size)
{
    if(size == 0 || p == nullptr) {
        return;
    }
    b3_assert(size > 0);

    if (size > b3_max_block_size) {
        b3_free(p);
        return;
    }

    // the block is actually not freed but just set to b3Block type
    int index = b3_size_map.m_values[size];
    b3_assert(index >= 0 && index < b3_block_size_count);

    b3Block *block = (b3Block *)p;

    block->m_next = m_free_lists[index];
    m_free_lists[index] = block;
}

/**
 * @brief clear all allocated blocks.
 */
void b3BlockAllocator::clear()
{
    for(int32 i = 0; i < m_chunk_count; ++i) {
        b3_free(m_chunks->m_blocks);
    }

    m_chunk_count = 0;
    memset(m_chunks, 0, m_chunk_space * sizeof(b3Chunk));
    memset(m_free_lists, 0, sizeof(m_free_lists));
}