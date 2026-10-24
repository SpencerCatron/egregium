#include <egregium/algorithms/heat_diffusion.h>
#include <egregium/operators/laplacian.h>

#include <stdexcept>

namespace egregium {

HeatDiffusion::HeatDiffusion(const Geometry& geom, const HeatParams& params = {}) {
    size_ = geom.mesh().nVertices();
    if (params.diffusivity.size() == 0) {
        diffusivity_ = Eigen::VectorXd::Ones(size_);
    } else if (params.diffusivity.size() != size_) {
        throw std::invalid_argument(
            "HeatDiffusion: Diffusivity must be same size as vertices on mesh.");
    } else {
        diffusivity_ = params.diffusivity;
    }
    timeStep_ = params.timeStep;
    ambient_ = params.ambient;
    coupling_ = params.coupling;
    L_ = cotanLaplacian(geom);
    M_ = massMatrix(geom);
    A_ = (1.0 + timeStep_ * coupling_) * M_ + timeStep_ * L_;

    solver_.compute(A_);
    if (solver_.info() != Eigen::Success) {
        throw std::runtime_error("HeatDiffusion: Factorization failed.");
    }
}

void HeatDiffusion::setState(const Eigen::VectorXd& u) {
    u_ = u;
}

void HeatDiffusion::setTimeStep(double timeStep) {
    timeStep_ = timeStep;
}

void HeatDiffusion::setAmbientTemp(double ambient, double coupling) {
    ambient_ = ambient;
    coupling_ = coupling;
    A_ = (1.0 + timeStep_ * coupling_) * M_ + timeStep_ * L_;
    solver_.compute(A_);
    if (solver_.info() != Eigen::Success) {
        throw std::runtime_error("HeatDiffusion: Factorization failed.");
    }
}

void HeatDiffusion::setDiffusivity(Eigen::VectorXd& diffusivity) {
    if (diffusivity.size() != size_) {
        throw std::invalid_argument(
            "HeatDiffusion: Diffusivity must be same size as vertices on mesh.");
    }
}

void HeatDiffusion::step() {
    timeTotal_ += timeStep_;
    u_ = solver_.solve(M_ * u_ +
                       timeStep_ * coupling_ * ambient_ * M_ * Eigen::VectorXd::Ones(size_));
}

double HeatDiffusion::elapsedTime() const {
    return timeTotal_;
}

Eigen::Vector3d HeatDiffusion::state() const {
    return u_;
}
}  // namespace egregium