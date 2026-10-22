#pragma once

#include <egregium/geometry/geometry.h>
#include <Eigen/Sparse>

namespace egregium{

Eigen::SparseMatrix<double> cotanLaplacian(const Geometry& geom);
Eigen::SparseMatrix<double> massMatrix(const Geometry& geom);
}