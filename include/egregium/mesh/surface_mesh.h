#pragma once

#include <array>
#include <vector>

namespace egregium {

constexpr int INVALID = -1;  // Placeholder for no input or boundary data.

using Faces = std::vector<std::array<int, 3>>;

// Halfedge structure for a manifold, consistently oriented triangle mesh.
//
// Every edge has two halfedges pointing in opposite directions.
// Each halfedge belongs to the face on its left side.
// Boundary halfedges are those that have no face on their left side.
//
// For indexing: the interior halfedges on face f are indexed 3f, 3f+1, 3f+2.
// Boundary halfedges are indexed after all interior halfedges in the order that we see interior
// halfedges without a twin.
class SurfaceMesh {
 public:
    // Builds the mesh from triangles listed in counterclockwise order. Vertex indices must be
    // non-negative and use every index between 0 and the max index.
    // Throws std::invalid_argument for empty input, negative indices, unused indices, non-manifold
    // mesh, or inconsistent orientation on faces.
    explicit SurfaceMesh(const Faces& faces);

    // -- Element counts ----------------------------------------------------

    int nVertices() const;
    int nEdges() const;
    int nFaces() const;
    int nHalfedges() const;

    // -- Halfedge navigation -----------------------------------------------

    int next(int h) const;  // Next halfedge on same face, counterclockwise.
    int twin(int h) const;  // Other halfedge along the same edge.
    int tailVertex(int h) const;
    int headVertex(int h) const;
    int edge(int h) const;
    int face(int h) const;  // Face to left of h, or INVALID on the boundary.

    // -- Elements to halfedge ----------------------------------------------

    int vertexHalfedge(int v) const;  // Sends vertex to choice of halfedge.
    int edgeHalfedge(int e) const;    // Sends edge to choice of halfedge.
    int faceHalfedge(int f) const;    // Sends face to choice of halfedge.

    // -- Derived queries ---------------------------------------------------

    bool isBoundaryHalfedge(int h) const;
    bool isBoundaryVertex(int v) const;
    int degree(int v) const;          // Number of edges at vertex v.
    int eulerCharacteristic() const;  // V-E+F, topological invariant.

 private:
    int buildInteriorHalfedges(const Faces& faces);
    void buildBoundaryHalfedges(const Faces& faces, int maxVertexIndex);
    void checkUnusedVertices() const;
    void checkNonManifoldVertices() const;  // Check for non-manifold interior vertices.
    int rotateHalfedge(int h) const;        // Rotate outgoing halfedge around its tail vertex.

    std::vector<int> next_;
    std::vector<int> twin_;
    std::vector<int> tail_;
    std::vector<int> edge_;
    std::vector<int> face_;

    std::vector<int> vertexHalfedge_;  // Sends vertex to choice of halfedge.
    std::vector<int> edgeHalfedge_;    // Sends edge to choice of halfedge.
    std::vector<int> faceHalfedge_;    // Sends face to choice of halfedge.
};

}  // namespace egregium
