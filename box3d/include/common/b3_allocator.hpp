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

#ifndef BOX3D_B3_ALLOCATOR_HPP
#define BOX3D_B3_ALLOCATOR_HPP


#include <malloc.h>
#include "common/b3_types.hpp"

/**
 * @brief allocate a block of memory of the specified size and return a pointer
 * to the beginning of this block.
 * @param size An integer specifying the number of bytes to allocate.
 * @return A pointer to the beginning of the allocated memory block.
 */
inline void* b3_alloc(int32 size) {
    return malloc(size);
}

/**
 * @brief  deallocate a previously allocated block of memory, freeing it up for
 * future use.
 * @param mem A pointer to the memory block that needs to be deallocated.
 */
inline void b3_free(void* mem) {
    free(mem);
}


#endif //BOX3D_B3_ALLOCATOR_HPP
