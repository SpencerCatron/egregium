# egregium
C++ library for geometry processing, discrete differential geometry, and more.

> **Status:** This project is in very early development.

## Features
- [x] Half-edge mesh data structure
- [x] Geometric data structre
- [x] Basic viewer (polyscope)
- [x] OBJ reading and writing
- [x] Mean and Gaussian curvatures
- [x] Cotangent Laplacian and mass matrix 

## Planned Features
- [ ] mesh generators for canonical surfaces
- [ ] Geodesic distance via heat method
- [ ] heat dispersion demo
- [ ] Minimal surfaces demo
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
