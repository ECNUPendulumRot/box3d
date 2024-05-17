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

下载代码，命令行命令
```
git clone https://github.com/ECNUPendulumRot/box3d.git
```
使用Visual Studio 2022打开该文件夹，在VS中打开CMakeLists.txt文件。
然后在启动项处选择该文档，运行即可。

### Building on Ubuntu
```
mkdir build && cd build
cmake ..
make
```
