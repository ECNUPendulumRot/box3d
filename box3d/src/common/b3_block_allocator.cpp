
#include <limits>

#include "common/b3_block_allocator.hpp"

#include "common/b3_allocator.hpp"

//TODO: this const value need modify

static const int32 b3_chunk_size = 16 * 1024;

static const int32 b3_max_block_size = 640;

static const int32 b3_chunk_array_increment = 128;


static const int32 b3_block_sizes[b3_block_size_count] = {
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


// This map an arbitrary allocation size to a suitable slot in b3_block_sizes
struct b3SizeMap {

    uint8 m_values[b3_max_block_size + 1];

    b3SizeMap() {
        int32 j = 0;
        m_values[0] = 0;

        for (int32 i = 1; i <= b3_max_block_size; ++i) {
            if(i > b3_block_sizes[j]) {
                ++j;
            }
            m_values[i] = (uint8)j;
        }
    }
};


static const b3SizeMap b3_size_map;


struct b3Chunk {
    b3Block* m_blocks;
};


struct b3Block {
    b3Block* m_next;
};


b3BlockAllocator::b3BlockAllocator() {

    b3_assert(b3_block_size_count < UCHAR_MAX);

    m_chunk_space = b3_chunk_array_increment;
    m_chunk_count = 0;
    m_chunks = (b3Chunk*)b3_alloc(m_chunk_space * sizeof(b3Chunk));

    memset(m_chunks, 0, m_chunk_space * sizeof(b3Chunk));
    memset(m_free_lists, 0, sizeof(m_free_lists));
}


b3BlockAllocator::~b3BlockAllocator() {
    for(int32 i = 0; i < m_chunk_count; ++i) {
        b3_free(m_chunks[i].m_blocks);
    }

    b3_free(m_chunks);
}


void* b3BlockAllocator::allocate(int32 size) {
    if(size == 0) {
        return nullptr;
    }

    b3_assert(size > 0);

    if(size > b3_max_block_size) {
        return b3_alloc(size);
    }

    int32 index = b3_size_map.m_values[size];
    b3_assert(index >= 0 && index < b3_block_size_count);

    if(m_free_lists[index]) {
        b3Block* block = m_free_lists[index];
        m_free_lists[index] = block->m_next;
        return block;
    } else {
        if(m_chunk_count == m_chunk_space) {
            b3Chunk* old_chunk = m_chunks;
            m_chunk_space += b3_chunk_array_increment;
            m_chunks = (b3Chunk*) b3_alloc(m_chunk_space * sizeof(b3Chunk));
            memcpy(m_chunks, old_chunk, m_chunk_count * sizeof(b3Chunk));
            memset(m_chunks + m_chunk_count, 0, b3_chunk_array_increment * sizeof(b3Chunk));
            b3_free(old_chunk);
        }

        b3Chunk* chunk = m_chunks + m_chunk_count;
        chunk->m_blocks = (b3Block*)b3_alloc(b3_chunk_size);
        int32 block_size = b3_block_sizes[index];
        int32 block_count = b3_chunk_size / block_size;

        b3_assert(block_count * block_size <= b3_chunk_size);

        int8* start = (int8*)chunk->m_blocks;
        for(int32 i = 0; i < block_count - 1; ++i) {
            // b3Block* block = (b3Block*)((int8*)chunk->m_blocks + block_size * i);
            b3Block* block = (b3Block*)(start);

            start += block_size;

            b3Block* next = (b3Block*)(start);
            // b3Block* next  = (b3Block*)((int8*)chunk->m_blocks + block_size * (i + 1));
            block->m_next = next;

        }
        b3Block* last = (b3Block*)((int8*)chunk->m_blocks + block_size * (block_count - 1));
        last->m_next = nullptr;

        m_free_lists[index] = chunk->m_blocks->m_next;
        ++m_chunk_count;

        return chunk->m_blocks;
    }
}


void b3BlockAllocator::free(void* p, int32 size) {
    if(size == 0 || p == nullptr) {
        return;
    }
    b3_assert(size > 0);

    if(size > b3_max_block_size) {
        b3_free(p);
        return;
    }

    int index = b3_size_map.m_values[size];
    b3_assert(index >= 0 && index < b3_block_size_count);

    b3Block* block = (b3Block*)p;
    block->m_next = m_free_lists[index];
    m_free_lists[index] = block;
}


void b3BlockAllocator::clear() {
    for(int32 i = 0; i < m_chunk_count; ++i) {
        b3_free(m_chunks->m_blocks);
    }

    m_chunk_count = 0;
    memset(m_chunks, 0, m_chunk_space * sizeof(b3Chunk));
    memset(m_free_lists, 0, sizeof(m_free_lists));
}