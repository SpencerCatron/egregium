#include <egregium/geometry/geometry.h>

namespace egregium {
Geometry::Geometry(const SurfaceMesh& mesh, std::vector<Eigen::Vector3d> positions)
    : mesh_(mesh), positions_(std::move(positions)) {
    if (static_cast<int>(positions_.size()) != mesh_.nVertices()) {
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
    if (static_cast<int>(newPositions.size()) != mesh_.nVertices()) {
        throw std::invalid_argument("Number of positions does not match number of vertices");
    }
    positions_ = std::move(newPositions);
}

Eigen::Vector3d Geometry::halfedgeVector(int h) const {
    int head = mesh().headVertex(h);
    int tail = mesh().tailVertex(h);
    return position(head) - position(tail);
}

double Geometry::edgeLength(int e) const {
    int h = mesh().edgeHalfedge(e);
    return halfedgeVector(h).norm();
}

double Geometry::cotanWeight(int e) const {
    int h = mesh().edgeHalfedge(e);
    return .5 * (cotan(h) + cotan(mesh().twin(h)));
}

double Geometry::cornerAngle(int h) const {
    if (mesh().isBoundaryHalfedge(h)) return 0;

    int h1 = mesh().twin(mesh().next(h));  // Correct orientation.
    int h2 = mesh().next(mesh().next(h));
    Eigen::Vector3d u(halfedgeVector(h1));
    Eigen::Vector3d v(halfedgeVector(h2));

    // Recall formulae for norm cross product and dot product
    return std::atan2(u.cross(v).norm(), u.dot(v));
}

double Geometry::cotan(int h) const {
    if (mesh().isBoundaryHalfedge(h)) return 0;

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

Eigen::Vector3d Geometry::faceNormal(int f) const {
    int h1 = mesh().faceHalfedge(f);
    int h2 = mesh().next(h1);
    Eigen::Vector3d u(halfedgeVector(h1));
    Eigen::Vector3d v(halfedgeVector(h2));
    return u.cross(v).normalized();
}

double Geometry::vertexDualArea(int v) const {
    int hStart = mesh().vertexHalfedge(v);
    int h = hStart;
    double areaSum = 0;
    do {
        if (!mesh().isBoundaryHalfedge(h)) {
            areaSum += faceArea(mesh().face(h)) / 3;
        }

        h = mesh().rotateHalfedge(h);
    } while (h != hStart);
    return areaSum;
}

Eigen::Vector3d Geometry::vertexNormal(int v) const {
    int hStart = mesh().vertexHalfedge(v);
    int h = hStart;
    Eigen::Vector3d vectorSum(0, 0, 0);
    do {
        if (!mesh().isBoundaryHalfedge(h)) {
            vectorSum += cornerAngle(mesh().next(h)) * faceNormal(mesh().face(h));
        }

        h = mesh().rotateHalfedge(h);
    } while (h != hStart);
    return vectorSum.normalized();
}

double Geometry::angleSum(int v) const {
    int hStart = mesh().vertexHalfedge(v);
    int h = hStart;
    double angleSum = 0;
    do {
        if (!mesh().isBoundaryHalfedge(h)) {
            angleSum += cornerAngle(mesh().next(h));
        }
        h = mesh().rotateHalfedge(h);

    } while (h != hStart);
    return angleSum;
}

double Geometry::angleDefect(int v) const {
    if (mesh().isBoundaryVertex(v)) {
        return 0;
    }
    return 2 * kPi - angleSum(v);
}

double Geometry::gaussian(int v) const{
    return angleDefect(v)/vertexDualArea(v);
}

double Geometry::boundaryTurningAngle(int v) const {
    if (!mesh().isBoundaryVertex(v)) {
        return 0;
    }
    return kPi - angleSum(v);
}

double Geometry::totalArea() const {
    double areaSum = 0;
    for (int f = 0; f < mesh().nFaces(); ++f) {
        areaSum += faceArea(f);
    }
    return areaSum;
}

double Geometry::meanEdgeLength() const {
    double edgeSum = 0;
    for (int e = 0; e < mesh().nEdges(); ++e) {
        edgeSum += edgeLength(e);
    }
    return edgeSum / mesh().nEdges();
}
}  // namespace egregium