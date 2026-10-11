#pragma once

#include <egregium/mesh/surface_mesh.h>

#include <Eigen/Dense>
#include <vector>

namespace egregium {
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

    double cornerAngle(int h) const;
    double cotan(int h) const;  

    // Faces

    double faceArea(int f) const;
    Eigen::Vector3d faceNormal(int f) const;

    // vertices

    double vertexDualArea(int v) const;
    Eigen::Vector3d vertexNormal(int v) const;
    double angleSum(int v) const;
    double angleDefect(int v) const;

    // Global

    double totalArea() const;
    double meanEdgeLength() const;

 private:
    const SurfaceMesh& mesh_;
    std::vector<Eigen::Vector3d> positions_;
};
}  // namespace egregium