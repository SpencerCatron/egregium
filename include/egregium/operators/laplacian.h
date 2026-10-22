#pragma once

#include <egregium/geometry/geometry.h>

#include <Eigen/Sparse>

namespace egregium {
// Returns the negative laplacian (positive semi-definite).
// L(i,i)= \sum_j W_ij , L(i,j)=- W_ij.
Eigen::SparseMatrix<double> cotanLaplacian(const Geometry& geom);
Eigen::SparseMatrix<double> massMatrix(const Geometry& geom);
}  // namespace egregium