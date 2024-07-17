# box3d

Box3D is a 3D physics engine for learning.

## External Library

under the extern/cmake folder

- Eigen: a C++ template library for linear algebra operations
- CLI11: a modern C++ command line parsing library
- Libigl: a C++ library for computational geometry
- Spdlog: a fast, asynchronous, header-only C++ logging library
- NlohmannJson: a modern C++ JSON library

## Features

### Geometry

- mesh
- cube
- sphere

### Math

- b3Vector
- b3Matrix


### Collision

- AABB bounding boxes
- Bounding volume hierarchy(BVH)
- Fixture is used to attach shapes to bodies

### Common

- Allocator: alloc and free memory space
- Common: some macro definitions
- TimeStep: the world time step
- Types: simplified definitions of some types

### Dynamics

- World: contacts all bodies and shapes.
- Body: has position and velocity. Bodies can be static, kinematic, or dynamic.
- Pose: express position or velocity.
- Inertia: used to compute objects mass and inertia.
- Island: speed up constraint processing.


### utils

- IO: read/write files
- Json: use Nlohmann_json to get data
- Log: use spdlog to logging
- Timer: use to timer

### Instruction for use

only need to include box3d.hpp

## Building

Install CMake

### Building on Windows

Download the code here and use the following command line commands
```
git clone https://github.com/ECNUPendulumRot/box3d.git
```
Open the folder using Visual Studio 2022, open the CMakeLists.txt file in VS, then select the document in the startup items and run it.

### Building on Ubuntu

Download the code here and use the following command line commands
```
mkdir build && cd build
cmake ..
make
```

## License

The MIT License

Copyright (c) 2024
Robot Motion and Vision Laboratory at East China Normal University
Contact: tophill.robotics@gmail.com

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
