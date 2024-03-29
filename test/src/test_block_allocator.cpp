
#include "box3d.hpp"

#include <iostream>

struct Test {
    int v[10];
};

int main() {
    b3BlockAllocator allocator;

    Test* t[5];

    for(int i = 0; i< 4; i++) {
        void* mem = allocator.allocate(sizeof(Test));
        t[i] = new(mem) Test;
        std::cout << "address: " << mem << std::endl;
    }

    for(int i = 3; i > 0; i--) {
        allocator.free(t[i], sizeof(Test));
    }

    for(int i = 1; i < 5; ++i) {
        void* mem = allocator.allocate(sizeof(Test));
        t[i] = new(mem) Test;
        std::cout << "address: " << mem << std::endl;
    }
}