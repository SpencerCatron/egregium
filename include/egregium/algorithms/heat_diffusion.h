#pragma once

#include <egregium/geometry/geometry.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace egregium {
struct HeatParams{
    double timeStep=0.01;
    Eigen::VectorXd diffusivity;
    double ambient=0.0;
    double coupling =0.0;
};

class HeatDiffusion {
 public:
    HeatDiffusion(const Geometry& geom, const HeatParams& params={});

    void setState(const Eigen::VectorXd& u);

    void setTimeStep(double timeStep);

    void setAmbientTemp(double ambient, double coupling);

    void setDiffusivity(Eigen::VectorXd& diffusivity);

    void step();

    double elapsedTime() const;

    Eigen::Vector3d state() const;

 private:
    Eigen::Vector3d u_;
    double timeStep_;
    double timeTotal_;
    Eigen::SparseMatrix<double> L_ ; // Cotan Laplacian
    Eigen::SparseMatrix<double> M_;  // Mass Matrix
    Eigen::VectorXd diffusivity_;
    double ambient_;
    double coupling_;
    int size_;

};
}  // namespace egregium