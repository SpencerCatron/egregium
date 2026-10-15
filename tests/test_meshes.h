#pragma once

#include <egregium/mesh/surface_mesh.h>

#include <Eigen/Dense>
#include <cmath>
#include <numbers>
#include <vector>

namespace test_meshes {

using egregium::Faces;
using Positions = std::vector<Eigen::Vector3d>;

// v=4,E=6, F=4, chi=2, closed
inline Faces tetrahedron() {
    return {{0, 1, 2}, {0, 3, 1}, {0, 2, 3}, {1, 3, 2}};
}

// V=6, E=12, F=8, chi=2, closed
inline Faces octahedron() {
    return {{0, 2, 4}, {2, 1, 4}, {1, 3, 4}, {3, 0, 4}, {2, 0, 5}, {1, 2, 5}, {3, 1, 5}, {0, 3, 5}};
}

// V=8, E=18, F=12, chi=2, closed
inline Faces cube() {
    return {{0, 2, 3}, {0, 3, 1}, {4, 5, 7}, {4, 7, 6}, {0, 1, 5}, {0, 5, 4},
            {2, 6, 7}, {2, 7, 3}, {0, 4, 6}, {0, 6, 2}, {1, 3, 7}, {1, 7, 5}};
}

// V=9, E=27, F=18, chi=0, closed
inline Faces torus() {
    return {{0, 3, 4}, {0, 4, 1}, {1, 4, 5}, {1, 5, 2}, {2, 5, 3}, {2, 3, 0},
            {3, 6, 7}, {3, 7, 4}, {4, 7, 8}, {4, 8, 5}, {5, 8, 6}, {5, 6, 3},
            {6, 0, 1}, {6, 1, 7}, {7, 1, 2}, {7, 2, 8}, {8, 2, 0}, {8, 0, 6}};
}

// V=3, E=3, F=1, chi=1. One boundary loop of length 3.
inline Faces singleTriangle() {
    return {{0, 1, 2}};
}

// V=4, E=5, F=2, chi=1. One boundary loop of length 4.
inline Faces square() {
    return {{0, 1, 2}, {0, 2, 3}};
}

// V=8, E=16, F=8, chi=0. Two boundary loops of length 4.
inline Faces annulus() {
    return {{0, 1, 5}, {0, 5, 4}, {1, 2, 6}, {1, 6, 5}, {2, 3, 7}, {2, 7, 6}, {3, 0, 4}, {3, 4, 7}};
}

// V=5, E=8, F=4, chi=1. One boundary loop of length 4.
inline Faces fan() {
    return {{0, 1, 4}, {1, 2, 4}, {2, 3, 4}, {3, 0, 4}};
}

// Two separate tetrahedra (vertices 0-3 and 4-7), no shared vertices.
// V=8, E=12, F=8, chi=4 (two spheres)
inline Faces twoTetrahedra() {
    return {{0, 1, 2}, {0, 3, 1}, {0, 2, 3}, {1, 3, 2}, {4, 5, 6}, {4, 7, 5}, {4, 6, 7}, {5, 7, 6}};
}

//------------------------------------------------------
// positions for the meshes above
//------------------------------------------------------

// Regular tetrahedron centered at the origin, edge length 2*sqrt(2).
inline Positions tetrahedronPositions() {
    return {{1, 1, 1}, {1, -1, -1}, {-1, 1, -1}, {-1, -1, 1}};
}

// Regular octahedron with vertices at +-x, +-y, +-z, edge length sqrt(2).
inline Positions octahedronPositions() {
    return {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
}

// Unit cube centered at the origin. Bit k of the vertex index is coordinate k.
inline Positions cubePositions() {
    Positions p;
    for (int i = 0; i < 8; ++i) {
        p.emplace_back((i & 1) - 0.5, ((i >> 1) & 1) - 0.5, ((i >> 2) & 1) - 0.5);
    }
    return p;
}

// Torus with major radius 2 and minor radius 1. Vertex 3*i+j sits at angle i around
// the major circle and angle j around the tube.
inline Positions torusPositions() {
    constexpr double kStep = 2.0 * std::numbers::pi / 3.0;
    Positions p;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            const double u = kStep * i;
            const double v = kStep * j;
            const double r = 2.0 + std::cos(v);
            p.emplace_back(r * std::cos(u), r * std::sin(u), std::sin(v));
        }
    }
    return p;
}

// Right isosceles triangle in the xy-plane with legs of length 1 along x and y.
inline Positions singleTrianglePositions() {
    return {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
}

// Unit square in the xy-plane, split along the diagonal 0-2.
inline Positions squarePositions() {
    return {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}};
}

// Square diamond rings of radius 2 (vertices 0-3) and 1 (vertices 4-7). Area 6.
inline Positions annulusPositions() {
    return {{2, 0, 0}, {0, 2, 0}, {-2, 0, 0}, {0, -2, 0},
            {1, 0, 0}, {0, 1, 0}, {-1, 0, 0}, {0, -1, 0}};
}

// Square [-1,1]^2 with the center vertex 4 lifted to the given height (0 is flat).
inline Positions fanPositions(double height = 0.0) {
    return {{-1, -1, 0}, {1, -1, 0}, {1, 1, 0}, {-1, 1, 0}, {0, 0, height}};
}

//------------------------------------------------------
// invalid input
//------------------------------------------------------

// Three triangles sharing edge 0-1 (non-manifold edge; repeats half-edge 1->0).
inline Faces nonManifoldEdge() {
    return {{0, 1, 2}, {1, 0, 3}, {1, 0, 4}};
}

// Two triangles touching only at vertex 0 (non-manifold vertex).
inline Faces bowtie() {
    return {{0, 1, 2}, {0, 3, 4}};
}

// Two triangles with opposite orientations sharing edge 0-1 (repeats 0->1).
inline Faces inconsistentOrientation() {
    return {{0, 1, 2}, {0, 1, 3}};
}

// Five-vertex Moebius strip ( repeats a half-edge).
inline Faces moebiusStrip() {
    return {{0, 1, 2}, {1, 2, 3}, {2, 3, 4}, {3, 4, 0}, {4, 0, 1}};
}

// Two tetrahedra sharing only vertex 0 (non-manifold interior vertex).
// No duplicate half-edges and no boundary, so only a vertex-rotation check catches it.
inline Faces tetrahedraSharingVertex() {
    return {{0, 1, 2}, {0, 3, 1}, {0, 2, 3}, {1, 3, 2}, {0, 4, 5}, {0, 6, 4}, {0, 5, 6}, {4, 6, 5}};
}

}  // namespace test_meshes