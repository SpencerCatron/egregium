#include <egregium/mesh/surface_mesh.h>

#include <algorithm>
#include <map>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>

namespace egregium {
namespace {
// check for negative indexing or repeated indexing on same face
void validateFaces(const Faces& faces) {
    if (faces.size() == 0) {
        throw std::invalid_argument("SurFaceMesh: input must be non-empty");
    }

    for (size_t i = 0; i < faces.size(); ++i) {
        const auto& f = faces[i];
        if (f[0] == f[1] || f[1] == f[2] || f[2] == f[0]) {
            throw std::invalid_argument("SurfaceMesh: invalid face at index " + std::to_string(i));
        }
        for (size_t j = 0; j < 3; ++j) {
            const int& v = f[j];
            if (v < 0) {
                throw std::invalid_argument("SurfaceMesh: negative vertex at index" +
                                            std::to_string(i) + ',' + std::to_string(j));
            }
        }
    }
}

}  // namespace

int SurfaceMesh::rotateHalfedge(int h) const {
    return next(twin(h));
}

void SurfaceMesh::checkNonManifoldVertices() const {
    std::vector<int> outgoing(nVertices(), 0);
    for (int h = 0; h < nHalfedges(); ++h) {
        ++outgoing[tailVertex(h)];
    }
    for (int v = 0; v < nVertices(); ++v) {
        if (degree(v) != outgoing[v]) {
            throw std::invalid_argument("SurfaceMesh: non manifold interior vertex at " +
                                        std::to_string(v));
        }
    }
}

int SurfaceMesh::buildInteriorHalfedges(const Faces& faces) {
    std::map<std::pair<int, int>, int> halfedgeIndex;
    int nEdges = 0;
    const int nFaces = static_cast<int>(faces.size());
    const int originalSize = 3 * nFaces;
    int maxVertexIndex = 0;

    next_.resize(originalSize);
    twin_.resize(originalSize, INVALID);
    tail_.resize(originalSize);
    edge_.resize(originalSize);
    face_.resize(originalSize);

    for (int i = 0; i < nFaces; ++i) {
        const auto& f = faces[i];

        maxVertexIndex = std::max({maxVertexIndex, f[0], f[1], f[2]});

        faceHalfedge_.push_back(3 * i);

        // iterate vertices
        for (int j = 0; j < 3; j++) {
            const int tail = f[j];
            const int head = f[(j + 1) % 3];
            const int heIdx = 3 * i + j;  // indexing for the current halfedge

            auto [it, inserted] = halfedgeIndex.try_emplace({tail, head}, heIdx);

            // handle bad orientation or non manifold behavior
            if (!inserted) {
                const auto& h = it->first;
                throw std::invalid_argument("SurfaceMesh: duplicate half-edge " +
                                            std::to_string(h.first) + ',' +
                                            std::to_string(h.second));
            }

            next_[heIdx] = 3 * i + (j + 1) % 3;
            tail_[heIdx] = tail;
            face_[heIdx] = i;

            // twin element hasn't been seen yet
            // count edge
            if (halfedgeIndex.count({head, tail}) == 0) {
                edge_[heIdx] = nEdges;
                edgeHalfedge_.push_back(heIdx);
                ++nEdges;
            }
            // twin has been seen.
            // same edge. declare twins
            else {
                int twinIdx = halfedgeIndex[{head, tail}];

                edge_[heIdx] = edge_[twinIdx];

                twin_[heIdx] = twinIdx;
                twin_[twinIdx] = heIdx;
            }
        }
    }
    return maxVertexIndex;
}

void SurfaceMesh::checkUnusedVertices() const {
    for (int i = 0; i < nVertices(); ++i) {
        if (vertexHalfedge(i) == INVALID) {
            throw std::invalid_argument("SurfaceMesh: invalid argument, unnused vertex at: " +
                                        std::to_string(i));
        }
    }
}

void SurfaceMesh::buildBoundaryHalfedges(const Faces& faces, int maxVertexIndex) {
    int boundaryCount = 0;
    std::unordered_map<int, int> heFromHead;
    const int nFaces = static_cast<int>(faces.size());
    const int originalSize = 3 * nFaces;
    for (int i = 0; i < originalSize; ++i) {
        if (twin_[i] == INVALID) {
            const int twinIdx = originalSize + boundaryCount;

            twin_[i] = twinIdx;
            // heFromHead[tail_[next_[i]]]=i;
            auto [it, inserted] = heFromHead.try_emplace(tail_[next_[i]], i);
            if (!inserted) {
                throw std::invalid_argument("SurfaceMesh: non-manifold boundary behavior at : " +
                                            std::to_string(tail_[next_[i]]));
            }

            face_.push_back(INVALID);
            twin_.push_back(i);
            edge_.push_back(edge_[i]);
            tail_.push_back(tail_[next_[i]]);
            next_.push_back(INVALID);

            boundaryCount++;
        }
    }
    for (const auto& [head, halfedgeIdx] : heFromHead) {
        const int twinIdx = twin_[halfedgeIdx];
        next_[twinIdx] = twin_[heFromHead.at(tail_[halfedgeIdx])];
    }

    // write vertexHalfedge_
    vertexHalfedge_.resize(maxVertexIndex + 1, INVALID);
    const int nHalfedges = static_cast<int>(tail_.size());
    for (int i = 0; i < nHalfedges; ++i) {
        vertexHalfedge_[tail_[i]] = i;
    }
}

SurfaceMesh::SurfaceMesh(const Faces& faces) {
    validateFaces(faces);
    int maxVertexIndex = buildInteriorHalfedges(faces);
    buildBoundaryHalfedges(faces, maxVertexIndex);

    // check for unnused vertices
    checkUnusedVertices();

    // check non manifold interior vertices
    checkNonManifoldVertices();
}

int SurfaceMesh::nVertices() const {
    return static_cast<int>(vertexHalfedge_.size());
}

int SurfaceMesh::nEdges() const {
    return static_cast<int>(edgeHalfedge_.size());
}

int SurfaceMesh::nFaces() const {
    return static_cast<int>(faceHalfedge_.size());
}

int SurfaceMesh::nHalfedges() const {
    return static_cast<int>(twin_.size());
}

int SurfaceMesh::next(int h) const {
    return next_[h];
}

int SurfaceMesh::twin(int h) const {
    return twin_[h];
}

int SurfaceMesh::tailVertex(int h) const {
    return tail_[h];
}

int SurfaceMesh::headVertex(int h) const {
    return tail_[next_[h]];
}

int SurfaceMesh::edge(int h) const {
    return edge_[h];
}

int SurfaceMesh::face(int h) const {
    return face_[h];
}

int SurfaceMesh::vertexHalfedge(int v) const {
    return vertexHalfedge_[v];
}

int SurfaceMesh::edgeHalfedge(int e) const {
    return edgeHalfedge_[e];
}

int SurfaceMesh::faceHalfedge(int f) const {
    return faceHalfedge_[f];
}

// derived
bool SurfaceMesh::isBoundaryHalfedge(int h) const {
    return face_[h] == INVALID;
}

bool SurfaceMesh::isBoundaryVertex(int v) const {
    return face_[vertexHalfedge_[v]] == INVALID;
}

int SurfaceMesh::degree(int v) const {
    int h = vertexHalfedge(v);
    int degreeCount = 1;
    int curH = rotateHalfedge(h);
    while (curH != h) {
        curH = rotateHalfedge(curH);
        ++degreeCount;
    }
    return degreeCount;
}

int SurfaceMesh::eulerCharacteristic() const {
    return SurfaceMesh::nVertices() - SurfaceMesh::nEdges() + SurfaceMesh::nFaces();
}
}  // namespace egregium