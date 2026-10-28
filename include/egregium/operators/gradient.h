#pragma once

#include <Eigen/Dense>
#include <egregium/geometry/geometry.h>

namespace egregium{

Eigen::MatrixXd gradient(const Geometry& geom, Eigen::VectorXd u);

}