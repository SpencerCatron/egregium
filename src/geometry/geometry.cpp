#include <egregium/geometry/geometry.h>

namespace egregium {
Geometry::Geometry(const SurfaceMesh& mesh, std::vector<Eigen::Vector3d> positions)
    : mesh_(mesh), positions_(positions) {
    if (positions_.size() != mesh_.nVertices()) {
        throw std::invalid_argument("Number of positions does not match number of vertices");
    }
}

const SurfaceMesh& Geometry::mesh() const {
    return mesh_;
}

const Eigen::Vector3d& Geometry::position(int v) const {
    return positions_[v];
}

void Geometry::setPositions(std::vector<Eigen::Vector3d> newPositions) {
    positions_ = newPositions;
}

Eigen::Vector3d Geometry::halfedgeVector(int h) const {
    int head = mesh().headVertex(h);
    int tail = mesh().tailVertex(h);
    return position(head) - position(tail);
}

double Geometry::edgeLength(int h) const {
    return halfedgeVector(h).norm();
}

double Geometry::cotanWeight(int e) const {
    int h = mesh().edgeHalfedge(e);
    return .5 * (cotan(h) + cotan(mesh().twin(h)));
}

double Geometry::cornerAngle(int h) const {
    int h1 = mesh().twin(mesh().next(h));  // Correct orientation.
    int h2 = mesh().next(mesh().next(h));
    Eigen::Vector3d u(halfedgeVector(h1));
    Eigen::Vector3d v(halfedgeVector(h2));

    // Recall formulae for norm cross product and dot product
    return std::atan2(u.cross(v).norm(), u.dot(v));
}

double Geometry::cotan(int h) const {
    int h1 = mesh().twin(mesh().next(h));  // Correct orientation.
    int h2 = mesh().next(mesh().next(h));
    Eigen::Vector3d u(halfedgeVector(h1));
    Eigen::Vector3d v(halfedgeVector(h2));

    return u.dot(v) / u.cross(v).norm();  // More precision than tan of cornerAngle.
}

double Geometry::faceArea(int f) const {
    int h = mesh().faceHalfedge(f);
    int h1 = mesh().next(h);
    int h2 = mesh().twin(h);  // Correct orientation.
    Eigen::Vector3d u(halfedgeVector(h1));
    Eigen::Vector3d v(halfedgeVector(h2));

    return .5 * u.cross(v).norm();  // Recall relation cross product and area of parallepiped.
}

Eigen::Vector3d Geometry::faceNormal(int f) const{
    int h1 = mesh().faceHalfedge(f);
    int h2 = mesh().next(h1);
    Eigen::Vector3d u(halfedgeVector(h1));
    Eigen::Vector3d v(halfedgeVector(h2));
    return u.cross(v).normalized();
}
}  // namespace egregium