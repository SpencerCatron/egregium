#pragma once

#include <egregium/geometry/geometry.h>
#include <egregium/mesh/surface_mesh.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace egregium {
struct mcfParams {
    double timeStep = 0.0005;
};

class MeanCurvatureFlow {
 public:
    // U_t = a \Delta u - k (u-T_a)
    //  ((1+tk)M+T_a L)u_new = M(u_old+tk T_a)
    // Uses reverse euler method to compute updated state
    MeanCurvatureFlow(const SurfaceMesh& mesh, Eigen::MatrixXd positions, const mcfParams& mcfParams = {});

    void setTimeStep(double timeStep);


    void step();

    double elapsedTime() const;

    Eigen::MatrixXd state() const;

 private:
    double timeStep_;
    double timeTotal_;
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver_;
    Geometry geom_;
};
}  // namespace egregium