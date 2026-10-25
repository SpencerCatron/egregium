#include <egregium/algorithms/mean_curvature_flow.h>
#include <egregium/operators/laplacian.h>

namespace egregium {

MeanCurvatureFlow::MeanCurvatureFlow(const SurfaceMesh& mesh,
                                     Eigen::MatrixXd positions,
                                     const mcfParams& mcfParams)
    : timeStep_(mcfParams.timeStep), timeTotal_(0.0), geom_(mesh, std::move(positions)) {
    solver_.analyzePattern(massMatrix(geom_) + timeStep_ * cotanLaplacian(geom_));
}

void MeanCurvatureFlow::setTimeStep(double timeStep) {
    timeStep_ = timeStep;
    solver_.analyzePattern(massMatrix(geom_) + timeStep_ * cotanLaplacian(geom_));
}

void MeanCurvatureFlow::step() {
    const Eigen::SparseMatrix<double> L = cotanLaplacian(geom_);
    const Eigen::SparseMatrix<double> M = massMatrix(geom_);
    const Eigen::SparseMatrix<double> A = M + timeStep_ * L;

    solver_.factorize(A);
    if (solver_.info() != Eigen::Success) {
        throw std::runtime_error("MeanCurvatureFlow: factorization failed");
    }
    Eigen::MatrixXd newPositions = solver_.solve(M*geom_.positionMatrix());
    if(solver_.info()!= Eigen::Success){
        throw std::runtime_error("MeanCurvatureFlow: Solving failed");
    }
    geom_.setPositions(newPositions);
    timeTotal_+=timeStep_;
}

double MeanCurvatureFlow::elapsedTime() const{
    return timeTotal_;
}

Eigen::MatrixXd MeanCurvatureFlow::state() const{
    return geom_.positionMatrix();
}
}  // namespace egregium