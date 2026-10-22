# egregium
C++ library for geometry processing, discrete differential geometry, and more.

> **Status:** This project is in very early development.

## Features
- [ ] Half-edge mesh data structure
- [ ] Geometric data strucutre
- [ ] Basic viewer (polyscope)
- [ ] OBJ reading and writing
- [ ] Mean and Gaussian curvatures

## Planned Features
- [ ] mesh generators for canonical surfaces
- [ ] Cotangent Laplacian and mass matrix
- [ ] Geodesic distance via heat method
- [ ] Minimal surfaces
- [ ] Better viewer
- [ ] Complete webapp version

## Building
Requires CMake 3.20+ and a C++20 compiler. Dependencies (Eigen, Polyscope, GoogleTest)
are fetched automatically by CMake.

```bash
git clone https://github.com/SpencerCatron/egregium.git
cd egregium
cmake -S . -B build
cmake --build build
ctest --test-dir build -C Debug
```
