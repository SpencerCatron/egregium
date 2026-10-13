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
class Geometry {
 public:
    explicit Geometry(const SurfaceMesh& mesh, std::vector<Eigen::Vector3d> positions);
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
    double angleDefect(int v) const;  // 2*pi - angle sum. This is the discrete Gaussian curvature.

    // Global

    double totalArea() const;
    double meanEdgeLength() const;

 private:
    const SurfaceMesh& mesh_;
    std::vector<Eigen::Vector3d> positions_;
};
}  // namespace egregium