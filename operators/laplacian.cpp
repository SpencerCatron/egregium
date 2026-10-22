#include <egregium/operators/laplacian.h>

namespace egregium {

// Build from triplets containing indexed position and value.
// Lu_i = \sum_j cotanweight(u_j-u_i)
Eigen::SparseMatrix<double> cotanLaplacian(const Geometry& geom) {
    std::vector<Eigen::Triplet<double>> triplets;
    const auto& mesh = geom.mesh();
    triplets.reserve(4 * mesh.nEdges());

    for (int e = 0; e < mesh.nEdges(); ++e) {
        int h = mesh.edgeHalfedge(e);
        double weight = geom.cotanWeight(h);
        int head = mesh.headVertex(h);
        int tail = mesh.tailVertex(h);

        triplets.emplace_back(tail, tail, -weight);
        triplets.emplace_back(tail, head, weight);
        triplets.emplace_back(head, head, -weight);
        triplets.emplace_back(head, tail, weight);
    }
    Eigen::SparseMatrix<double> L(mesh.nVertices(), mesh.nVertices());
    L.setFromTriplets(triplets.begin(), triplets.end());
    return L;
}

Eigen::SparseMatrix<double> massMatrix(const Geometry& geom) {
    std::vector<Eigen::Triplet<double>> triplets;
    const auto& mesh = geom.mesh();
    triplets.reserve( mesh.nVertices());
    
    for(int v=0; v<mesh.nVertices(); ++v){
        triplets.emplace_back(v,v, geom.vertexDualArea(v));
    }
    Eigen::SparseMatrix<double> M(mesh.nVertices(), mesh.nVertices());
    M.setFromTriplets(triplets.begin(), triplets.end());
    return M;
}
}  // namespace egregium