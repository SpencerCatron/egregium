#pragma once

#include <egregium/geometry/geometry.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace egregium {
struct HeatParams {
    double timeStep = 0.0001;
    Eigen::VectorXd diffusivity;
    double ambient = 0.0;
    double coupling = 0.0;
};

class HeatDiffusion {
 public:
    // U_t = a \Delta u - k (u-T_a)
    //  ((1+tk)M+T_a L)u_new = M(u_old+tk T_a)
    // Uses reverse euler method to compute updated state
    // Throws invalid_argument if the diffusion vector is a different size from nVertices.
    HeatDiffusion(const Geometry& geom, const HeatParams& params = {});

    void setState(const Eigen::VectorXd& u);

    void setTimeStep(double timeStep);

    void setAmbientTemp(double ambient, double coupling);

    void setDiffusivity(Eigen::VectorXd& diffusivity);

    void step();
    void step(Eigen::VectorXd& forcing);//overload to step with a forcing term

    double elapsedTime() const;

    Eigen::VectorXd state() const;

 private:
    Eigen::VectorXd u_;
    double timeStep_;
    double timeTotal_;
    Eigen::SparseMatrix<double> L_;  // Cotan Laplacian
    Eigen::SparseMatrix<double> M_;  // Mass Matrix
    Eigen::SparseMatrix<double> A_;
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver_;
    Eigen::VectorXd diffusivity_;
    double ambient_;
    double coupling_;
    int size_;
};
}  // namespace egregium