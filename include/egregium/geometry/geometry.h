#pragma once

#include <egregium/mesh/surface_mesh.h>

#include <Eigen/Dense>
#include <numbers>
#include <vector>

namespace egregium {
constexpr double kPi = std::numbers::pi;

// Geometric structure for a manifold, consistently oriented triangle mesh.
//
// Geometry is encoded entirely by positions of each vertex. From positions, we get lengths, angles,
// areas, normals, and curvature.
//
// Geometry holds a reference to the mesh, which must outlive it, and positions.
//
// Angles are radians, halfedge quantities return 0 if they are boundary.
class Geometry {
 public:
    // positions[v] is the position of vertex v.
    // Mesh must outlive Geometry.
    // Throws std::invalid_argument if sizes of mesh and positions are not equal.
    explicit Geometry(const SurfaceMesh& mesh, std::vector<Eigen::Vector3d> positions);

    // Temporary meshes are rejected at compile time, since mesh_ would dangle.
    explicit Geometry(SurfaceMesh&& mesh, std::vector<Eigen::Vector3d> positions) = delete;
    const SurfaceMesh& mesh() const;
    const Eigen::Vector3d& position(int v) const;
    void setPositions(std::vector<Eigen::Vector3d> newPositions);
    Eigen::Vector3d halfedgeVector(int h) const;

    // edges

    double edgeLength(int e) const;
    double cotanWeight(int e) const;  // .5(cotan of both opposite angels).

    // Corners

    double cornerAngle(int h) const;  // Angle opposite of given halfedge.
    double cotan(int h) const;        // Cotangent of angle opposite of given halfedge.

    // Faces

    double faceArea(int f) const;
    Eigen::Vector3d faceNormal(int f) const;

    // vertices

    double vertexDualArea(int v) const;  // Barycentric area : 1/3 area of each touching face.
    Eigen::Vector3d vertexNormal(int v) const;  // Angle weighted mean of normals surrounding v.
    double angleSum(int v) const;

    // 2*pi - angleSum. This is the discrete Gaussian curvature.
    // Returns 0 for boundary vertices.
    double angleDefect(int v) const;

    // pi - angleSum. This is the geodesic curvature of the boundary curve.
    // Returns 0 for interior vertices.
    double boundaryTurningAngle(int v) const;

    // Global

    double totalArea() const;
    double meanEdgeLength() const;

 private:
    const SurfaceMesh& mesh_;
    std::vector<Eigen::Vector3d> positions_;
};  
}  // namespace egregium