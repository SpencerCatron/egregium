#include <egregium/geometry/geometry.h>

#include <Eigen/Dense>

namespace egregium {

// ∇u_f = 1/(2A_f) · Σ_i  u_i · (N_f × e_i)
Eigen::MatrixXd gradient(const Geometry& geom, const Eigen::VectorXd u) {
    Eigen::MatrixXd output(geom.mesh().nFaces(), 3);

    const SurfaceMesh& mesh = geom.mesh();

    for(int f =0; f<mesh.nFaces(); ++f){
        Eigen::Vector3d faceGradient = Eigen::Vector3d::Zero();
        int h = mesh.faceHalfedge(f);
        double area = geom.faceArea(f);
        Eigen::Vector3d faceNormal = geom.faceNormal(f);
        do{
            Eigen::Vector3d u = geom.halfedgeVector(h);
            int oppositeVertex = mesh.headVertex(  mesh.next(h));
            faceGradient+=  u(oppositeVertex) * faceNormal.cross( u) / (2* area);
            h=mesh.next(h);
        }while(h!=h);
        output.row(f) = faceGradient.transpose();
    }
    return output;
}
}  // namespace egregium