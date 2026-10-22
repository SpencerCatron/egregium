#include <egregium/geometry/geometry.h>
#include <egregium/mesh/surface_mesh.h>
#include <egregium/operators/laplacian.h>
#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <cmath>
#include <ostream>
#include <random>
#include <string>

#include "test_meshes.h"

using egregium::Geometry;
using egregium::SurfaceMesh;
using test_meshes::Positions;

namespace {

constexpr double kTol = 1e-12;

// Stacks positions into an n x 3 matrix so L can act on all three coordinates at once.
Eigen::MatrixXd toMatrix(const Positions& positions) {
    Eigen::MatrixXd P(positions.size(), 3);
    for (int i = 0; i < static_cast<int>(positions.size()); ++i) {
        P.row(i) = positions[i].transpose();
    }
    return P;
}

}  // namespace

// testing class. mesh with positions.
struct OperatorCase {
    std::string name;
    test_meshes::Faces faces;
    Positions positions;
};

void PrintTo(const OperatorCase& c, std::ostream* os) {
    *os << c.name;
}

class OperatorTest : public ::testing::TestWithParam<OperatorCase> {};

TEST_P(OperatorTest, LaplacianHasRightSize) {
    SurfaceMesh mesh(GetParam().faces);
    Geometry geom(mesh, GetParam().positions);
    const Eigen::SparseMatrix<double> L = egregium::cotanLaplacian(geom);
    EXPECT_EQ(L.rows(), mesh.nVertices());
    EXPECT_EQ(L.cols(), mesh.nVertices());
}

TEST_P(OperatorTest, LaplacianIsSymmetric) {
    SurfaceMesh mesh(GetParam().faces);
    Geometry geom(mesh, GetParam().positions);
    const Eigen::SparseMatrix<double> L = egregium::cotanLaplacian(geom);
    const Eigen::SparseMatrix<double> Lt = L.transpose();
    EXPECT_NEAR((L - Lt).norm(), 0.0, kTol);
}

// Each edge ij contributes -w_ij off the diagonal, so L(i,j) is minus the cotan weight.
TEST_P(OperatorTest, OffDiagonalEntriesAreNegatedCotanWeights) {
    SurfaceMesh mesh(GetParam().faces);
    Geometry geom(mesh, GetParam().positions);
    const Eigen::SparseMatrix<double> L = egregium::cotanLaplacian(geom);
    for (int e = 0; e < mesh.nEdges(); ++e) {
        const int h = mesh.edgeHalfedge(e);
        const int i = mesh.tailVertex(h);
        const int j = mesh.headVertex(h);
        EXPECT_NEAR(L.coeff(i, j), -geom.cotanWeight(e), kTol) << "edge " << e;
    }
}

// Constants are in the null space: every row sums to zero.
TEST_P(OperatorTest, RowsSumToZero) {
    SurfaceMesh mesh(GetParam().faces);
    Geometry geom(mesh, GetParam().positions);
    const Eigen::SparseMatrix<double> L = egregium::cotanLaplacian(geom);
    const Eigen::VectorXd ones = Eigen::VectorXd::Ones(mesh.nVertices());
    EXPECT_NEAR((L * ones).norm(), 0.0, kTol);
}

// The header documents L as positive semidefinite: x^T L x >= 0 for every x.
TEST_P(OperatorTest, LaplacianIsPositiveSemidefinite) {
    SurfaceMesh mesh(GetParam().faces);
    Geometry geom(mesh, GetParam().positions);
    const Eigen::SparseMatrix<double> L = egregium::cotanLaplacian(geom);

    std::mt19937 rng(42);
    std::uniform_real_distribution<double> dist(-1.0, 1.0);
    for (int trial = 0; trial < 10; ++trial) {
        Eigen::VectorXd x(mesh.nVertices());
        for (int i = 0; i < x.size(); ++i) {
            x[i] = dist(rng);
        }
        EXPECT_GE(x.dot(L * x), -kTol) << "trial " << trial;
    }
}

TEST_P(OperatorTest, MassMatrixIsDiagonalOfDualAreas) {
    SurfaceMesh mesh(GetParam().faces);
    Geometry geom(mesh, GetParam().positions);
    const Eigen::SparseMatrix<double> M = egregium::massMatrix(geom);

    ASSERT_EQ(M.rows(), mesh.nVertices());
    ASSERT_EQ(M.cols(), mesh.nVertices());
    EXPECT_EQ(M.nonZeros(), mesh.nVertices());
    for (int v = 0; v < mesh.nVertices(); ++v) {
        EXPECT_NEAR(M.coeff(v, v), geom.vertexDualArea(v), kTol) << "vertex " << v;
    }
}

TEST_P(OperatorTest, MassMatrixSumsToTotalArea) {
    SurfaceMesh mesh(GetParam().faces);
    Geometry geom(mesh, GetParam().positions);
    const Eigen::SparseMatrix<double> M = egregium::massMatrix(geom);
    EXPECT_NEAR(M.sum(), geom.totalArea(), kTol);
}

INSTANTIATE_TEST_SUITE_P(
    AllMeshes, OperatorTest,
    ::testing::Values(
        OperatorCase{"tetrahedron", test_meshes::tetrahedron(), test_meshes::tetrahedronPositions()},
        OperatorCase{"octahedron", test_meshes::octahedron(), test_meshes::octahedronPositions()},
        OperatorCase{"cube", test_meshes::cube(), test_meshes::cubePositions()},
        OperatorCase{"torus", test_meshes::torus(), test_meshes::torusPositions()},
        OperatorCase{"triangle", test_meshes::singleTriangle(),
                     test_meshes::singleTrianglePositions()},
        OperatorCase{"square", test_meshes::square(), test_meshes::squarePositions()},
        OperatorCase{"pyramid", test_meshes::fan(), test_meshes::fanPositions(1.0)},
        OperatorCase{"annulus", test_meshes::annulus(), test_meshes::annulusPositions()}),
    [](const ::testing::TestParamInfo<OperatorCase>& info) { return info.param.name; });

// -- Known values ------------------------------------------------------------

// Property tests pass for any positive multiple of L, so pin down the scale exactly.
// Both angles opposite the diagonal 0-2 are right angles, so its weight is 0. Each
// boundary edge has one 45 degree angle opposite it, so its weight is cot(45)/2 = 1/2.
TEST(LaplacianSquare, KnownEntries) {
    SurfaceMesh mesh(test_meshes::square());
    Geometry geom(mesh, test_meshes::squarePositions());
    const Eigen::SparseMatrix<double> L = egregium::cotanLaplacian(geom);

    EXPECT_NEAR(L.coeff(0, 2), 0.0, kTol);
    EXPECT_NEAR(L.coeff(0, 1), -0.5, kTol);
    EXPECT_NEAR(L.coeff(1, 2), -0.5, kTol);
    EXPECT_NEAR(L.coeff(0, 0), 1.0, kTol);  // two boundary edges, plus the diagonal's 0
    EXPECT_NEAR(L.coeff(1, 1), 1.0, kTol);
    EXPECT_NEAR(L.coeff(1, 3), 0.0, kTol);  // not an edge
}

// The cotan Laplacian is exact for linear functions on a flat mesh: at an interior
// vertex, L applied to x, y or any linear combination is zero.
TEST(LaplacianFlat, LinearFunctionsAreHarmonicAtInteriorVertices) {
    SurfaceMesh mesh(test_meshes::fan());
    Geometry geom(mesh, test_meshes::fanPositions());
    const Eigen::SparseMatrix<double> L = egregium::cotanLaplacian(geom);
    const Eigen::MatrixXd LP = L * toMatrix(test_meshes::fanPositions());

    const int center = 4;
    ASSERT_FALSE(mesh.isBoundaryVertex(center));
    EXPECT_NEAR(LP.row(center).norm(), 0.0, kTol);
}

// L applied to the positions is the mean curvature normal. On a convex mesh it points
// outward along the vertex normal. Its length is a different discretization of mean
// curvature from the dihedral one, so only the direction is compared.
TEST(LaplacianOctahedron, PositionsGiveOutwardNormals) {
    SurfaceMesh mesh(test_meshes::octahedron());
    Geometry geom(mesh, test_meshes::octahedronPositions());
    const Eigen::SparseMatrix<double> L = egregium::cotanLaplacian(geom);
    const Eigen::MatrixXd LP = L * toMatrix(test_meshes::octahedronPositions());

    for (int v = 0; v < mesh.nVertices(); ++v) {
        const Eigen::Vector3d meanCurvatureNormal = LP.row(v).transpose();
        ASSERT_GT(meanCurvatureNormal.norm(), 0.0) << "vertex " << v;
        EXPECT_TRUE(meanCurvatureNormal.normalized().isApprox(geom.vertexNormal(v)))
            << "vertex " << v;
    }
}

// Every edge weight is cot(60)/2 + cot(60)/2 = 1/sqrt(3), and each vertex has 4 edges.
// The neighbors of vertex v are at distance sqrt(2) and sum to zero, so
// (L P)_v = 4 * (1/sqrt(3)) * p_v.
TEST(LaplacianOctahedron, KnownMeanCurvatureNormal) {
    SurfaceMesh mesh(test_meshes::octahedron());
    Geometry geom(mesh, test_meshes::octahedronPositions());
    const Eigen::SparseMatrix<double> L = egregium::cotanLaplacian(geom);
    const Eigen::MatrixXd LP = L * toMatrix(test_meshes::octahedronPositions());

    for (int v = 0; v < mesh.nVertices(); ++v) {
        const Eigen::Vector3d expected = 4.0 / std::sqrt(3.0) * geom.position(v);
        EXPECT_TRUE(LP.row(v).transpose().isApprox(expected)) << "vertex " << v;
    }
}
