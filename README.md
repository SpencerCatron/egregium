# egregium
C++ library for geometry processing, discrete differential geometry, and more.

> **Status:** This project is in very early development.

## Features
- [ ] Half-edge mesh data structure
- [ ] Geometric data strucutre

## Planned Features
- [ ] OBJ loading and mesh generators for canonical surfaces
- [ ] Cotangent Laplacian and mass matrix
- [ ] Mean curvature. Convergence tests
- [ ] Geodesic distance via heat method
- [ ] Mean curvature flow
- [ ] Minimal surfaces
- [ ] Interactive viewer (Polyscope)

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
