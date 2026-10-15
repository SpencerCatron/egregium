#include <egregium/geometry/geometry.h>
#include <egregium/mesh/surface_mesh.h>
#include <gtest/gtest.h>

#include <cmath>
#include <ostream>
#include <stdexcept>
#include <string>
#include <type_traits>

#include "test_meshes.h"

using egregium::Geometry;
using egregium::kPi;
using egregium::SurfaceMesh;
using test_meshes::Positions;

namespace {

constexpr double kTol = 1e-12;

// A temporary mesh must not bind to Geometry, but a named one must.
static_assert(!std::is_constructible_v<Geometry, SurfaceMesh&&, Positions>);
static_assert(std::is_constructible_v<Geometry, const SurfaceMesh&, Positions>);

}  // namespace

// testing class. mesh with positions and its known total area.
struct GeometryCase {
    std::string name;
    test_meshes::Faces faces;
    Positions positions;
    double area;
    bool closed;
};

void PrintTo(const GeometryCase& c, std::ostream* os) {
    *os << c.name;
}

class GeometryTest : public ::testing::TestWithParam<GeometryCase> {};

TEST_P(GeometryTest, TotalArea) {
    const GeometryCase& c = GetParam();
    SurfaceMesh mesh(c.faces);
    Geometry geom(mesh, c.positions);
    EXPECT_NEAR(geom.totalArea(), c.area, kTol);
}

TEST_P(GeometryTest, DualAreasSumToTotalArea) {
    SurfaceMesh mesh(GetParam().faces);
    Geometry geom(mesh, GetParam().positions);
    double sum = 0.0;
    for (int v = 0; v < mesh.nVertices(); ++v) {
        sum += geom.vertexDualArea(v);
    }
    EXPECT_NEAR(sum, geom.totalArea(), kTol);
}

TEST_P(GeometryTest, EdgeLengthIsDistanceBetweenEndpoints) {
    SurfaceMesh mesh(GetParam().faces);
    Geometry geom(mesh, GetParam().positions);
    for (int e = 0; e < mesh.nEdges(); ++e) {
        const int h = mesh.edgeHalfedge(e);
        const double expected =
            (geom.position(mesh.headVertex(h)) - geom.position(mesh.tailVertex(h))).norm();
        EXPECT_NEAR(geom.edgeLength(e), expected, kTol) << "edge " << e;
    }
}

TEST_P(GeometryTest, CornerAnglesOfFaceSumToPi) {
    SurfaceMesh mesh(GetParam().faces);
    Geometry geom(mesh, GetParam().positions);
    for (int f = 0; f < mesh.nFaces(); ++f) {
        const int h = mesh.faceHalfedge(f);
        const double sum = geom.cornerAngle(h) + geom.cornerAngle(mesh.next(h)) +
                           geom.cornerAngle(mesh.next(mesh.next(h)));
        EXPECT_NEAR(sum, kPi, kTol) << "face " << f;
    }
}

TEST_P(GeometryTest, CotanMatchesCornerAngle) {
    SurfaceMesh mesh(GetParam().faces);
    Geometry geom(mesh, GetParam().positions);
    for (int h = 0; h < mesh.nHalfedges(); ++h) {
        if (mesh.isBoundaryHalfedge(h)) {
            EXPECT_EQ(geom.cornerAngle(h), 0.0) << "half-edge " << h;
            EXPECT_EQ(geom.cotan(h), 0.0) << "half-edge " << h;
        } else {
            const double angle = geom.cornerAngle(h);
            EXPECT_NEAR(geom.cotan(h), std::cos(angle) / std::sin(angle), 1e-9)
                << "half-edge " << h;
        }
    }
}

TEST_P(GeometryTest, FaceNormalsAreUnitAndPerpendicular) {
    SurfaceMesh mesh(GetParam().faces);
    Geometry geom(mesh, GetParam().positions);
    for (int f = 0; f < mesh.nFaces(); ++f) {
        const Eigen::Vector3d n = geom.faceNormal(f);
        const int h = mesh.faceHalfedge(f);
        EXPECT_NEAR(n.norm(), 1.0, kTol) << "face " << f;
        EXPECT_NEAR(n.dot(geom.halfedgeVector(h)), 0.0, kTol) << "face " << f;
        EXPECT_NEAR(n.dot(geom.halfedgeVector(mesh.next(h))), 0.0, kTol) << "face " << f;
    }
}

TEST_P(GeometryTest, VertexNormalsAreUnit) {
    SurfaceMesh mesh(GetParam().faces);
    Geometry geom(mesh, GetParam().positions);
    for (int v = 0; v < mesh.nVertices(); ++v) {
        EXPECT_NEAR(geom.vertexNormal(v).norm(), 1.0, kTol) << "vertex " << v;
    }
}

// Discrete Gauss-Bonnet: total angle defect of a closed mesh is exactly 2*pi*chi.
TEST_P(GeometryTest, GaussBonnet) {
    const GeometryCase& c = GetParam();
    SurfaceMesh mesh(c.faces);
    Geometry geom(mesh, c.positions);
    double sum = 0.0;
    for (int v = 0; v < mesh.nVertices(); ++v) {
        sum += geom.angleDefect(v)+geom.boundaryTurningAngle(v);
    }
    EXPECT_NEAR(sum, 2.0 * kPi * mesh.eulerCharacteristic(), 1e-10);
}

// Scaling by s multiplies lengths by s and areas by s^2, and leaves angles unchanged.
TEST_P(GeometryTest, ScalingBehavior) {
    const GeometryCase& c = GetParam();
    SurfaceMesh mesh(c.faces);
    Geometry geom(mesh, c.positions);
    Geometry scaled(mesh, c.positions);

    constexpr double kScale = 2.5;
    Positions scaledPositions = c.positions;
    for (auto& p : scaledPositions) {
        p *= kScale;
    }
    scaled.setPositions(scaledPositions);

    EXPECT_NEAR(scaled.totalArea(), kScale * kScale * geom.totalArea(), 1e-10);
    EXPECT_NEAR(scaled.meanEdgeLength(), kScale * geom.meanEdgeLength(), 1e-10);
    for (int h = 0; h < mesh.nHalfedges(); ++h) {
        EXPECT_NEAR(scaled.cornerAngle(h), geom.cornerAngle(h), kTol) << "half-edge " << h;
    }
}

INSTANTIATE_TEST_SUITE_P(
    AllGeometries, GeometryTest,
    ::testing::Values(
        GeometryCase{"tetrahedron", test_meshes::tetrahedron(), test_meshes::tetrahedronPositions(),
                     8.0 * std::sqrt(3.0), true},
        GeometryCase{"octahedron", test_meshes::octahedron(), test_meshes::octahedronPositions(),
                     4.0 * std::sqrt(3.0), true},
        GeometryCase{"cube", test_meshes::cube(), test_meshes::cubePositions(), 6.0, true},
        GeometryCase{"triangle", test_meshes::singleTriangle(),
                     test_meshes::singleTrianglePositions(), 0.5, false},
        GeometryCase{"square", test_meshes::square(), test_meshes::squarePositions(), 1.0, false},
        GeometryCase{"fan", test_meshes::fan(), test_meshes::fanPositions(), 4.0, false},
        GeometryCase{"annulus", test_meshes::annulus(), test_meshes::annulusPositions(), 6.0,
                     false}),
    [](const ::testing::TestParamInfo<GeometryCase>& info) { return info.param.name; });

// The torus has no simple closed-form area, so it only gets the topological check.
TEST(GeometryTorus, GaussBonnet) {
    SurfaceMesh mesh(test_meshes::torus());
    Geometry geom(mesh, test_meshes::torusPositions());
    double sum = 0.0;
    for (int v = 0; v < mesh.nVertices(); ++v) {
        sum += geom.angleDefect(v);
    }
    EXPECT_NEAR(sum, 0.0, 1e-10);
}

// -- Known values ----------------------------------------------------------

// Halfedges 0, 1, 2 run 0->1, 1->2, 2->0, and edge i is the edge of halfedge i.
TEST(GeometryRightTriangle, KnownValues) {
    SurfaceMesh mesh(test_meshes::singleTriangle());
    Geometry geom(mesh, test_meshes::singleTrianglePositions());

    EXPECT_NEAR(geom.faceArea(0), 0.5, kTol);
    EXPECT_TRUE(geom.faceNormal(0).isApprox(Eigen::Vector3d::UnitZ()));

    // Opposite corners: halfedge 0 -> vertex 2, halfedge 1 -> vertex 0, halfedge 2 -> vertex 1.
    EXPECT_NEAR(geom.cornerAngle(0), kPi / 4, kTol);
    EXPECT_NEAR(geom.cornerAngle(1), kPi / 2, kTol);
    EXPECT_NEAR(geom.cornerAngle(2), kPi / 4, kTol);
    EXPECT_NEAR(geom.cotan(0), 1.0, kTol);
    EXPECT_NEAR(geom.cotan(1), 0.0, kTol);
    EXPECT_NEAR(geom.cotan(2), 1.0, kTol);

    // Boundary edges have only one interior angle, so the weight is half its cotan.
    EXPECT_NEAR(geom.cotanWeight(mesh.edge(0)), 0.5, kTol);
    EXPECT_NEAR(geom.cotanWeight(mesh.edge(1)), 0.0, kTol);

    EXPECT_NEAR(geom.edgeLength(mesh.edge(0)), 1.0, kTol);
    EXPECT_NEAR(geom.edgeLength(mesh.edge(1)), std::sqrt(2.0), kTol);
    EXPECT_NEAR(geom.edgeLength(mesh.edge(2)), 1.0, kTol);
    EXPECT_NEAR(geom.meanEdgeLength(), (2.0 + std::sqrt(2.0)) / 3.0, kTol);

    for (int v = 0; v < 3; ++v) {
        EXPECT_NEAR(geom.vertexDualArea(v), 0.5 / 3.0, kTol) << "vertex " << v;
        EXPECT_TRUE(geom.vertexNormal(v).isApprox(Eigen::Vector3d::UnitZ())) << "vertex " << v;
    }
    EXPECT_NEAR(geom.angleSum(0), kPi / 2, kTol);
}

// Both angles opposite the diagonal are right angles, so its cotan weight is 0.
TEST(GeometrySquare, DiagonalCotanWeightIsZero) {
    SurfaceMesh mesh(test_meshes::square());
    Geometry geom(mesh, test_meshes::squarePositions());
    int interiorEdges = 0;
    for (int e = 0; e < mesh.nEdges(); ++e) {
        const int h = mesh.edgeHalfedge(e);
        if (!mesh.isBoundaryHalfedge(h) && !mesh.isBoundaryHalfedge(mesh.twin(h))) {
            EXPECT_NEAR(geom.cotanWeight(e), 0.0, kTol);
            EXPECT_NEAR(geom.edgeLength(e), std::sqrt(2.0), kTol);
            ++interiorEdges;
        }
    }
    EXPECT_EQ(interiorEdges, 1);
}

// Every face is equilateral with side sqrt(2), and four faces meet at each vertex.
TEST(GeometryOctahedron, KnownValues) {
    SurfaceMesh mesh(test_meshes::octahedron());
    Geometry geom(mesh, test_meshes::octahedronPositions());

    for (int e = 0; e < mesh.nEdges(); ++e) {
        EXPECT_NEAR(geom.edgeLength(e), std::sqrt(2.0), kTol) << "edge " << e;
        EXPECT_NEAR(geom.cotanWeight(e), 1.0 / std::sqrt(3.0), kTol) << "edge " << e;
    }
    EXPECT_NEAR(geom.meanEdgeLength(), std::sqrt(2.0), kTol);

    for (int v = 0; v < mesh.nVertices(); ++v) {
        EXPECT_NEAR(geom.angleSum(v), 4.0 * kPi / 3.0, kTol) << "vertex " << v;
        EXPECT_NEAR(geom.angleDefect(v), 2.0 * kPi / 3.0, kTol) << "vertex " << v;
        EXPECT_NEAR(geom.vertexDualArea(v), 2.0 * std::sqrt(3.0) / 3.0, kTol) << "vertex " << v;
        // By symmetry the normal points straight out along the vertex position.
        EXPECT_TRUE(geom.vertexNormal(v).isApprox(geom.position(v))) << "vertex " << v;
    }
}

// On a convex mesh around the origin, counterclockwise faces have outward normals.
TEST(GeometryConvex, FaceNormalsPointOutward) {
    const std::vector<std::pair<test_meshes::Faces, Positions>> cases = {
        {test_meshes::tetrahedron(), test_meshes::tetrahedronPositions()},
        {test_meshes::octahedron(), test_meshes::octahedronPositions()},
        {test_meshes::cube(), test_meshes::cubePositions()},
    };
    for (const auto& [faces, positions] : cases) {
        SurfaceMesh mesh(faces);
        Geometry geom(mesh, positions);
        for (int f = 0; f < mesh.nFaces(); ++f) {
            const int h = mesh.faceHalfedge(f);
            const Eigen::Vector3d centroid =
                (geom.position(mesh.tailVertex(h)) + geom.position(mesh.headVertex(h)) +
                 geom.position(mesh.headVertex(mesh.next(h)))) /
                3.0;
            EXPECT_GT(geom.faceNormal(f).dot(centroid), 0.0) << "face " << f;
        }
    }
}

TEST(GeometryFan, FlatCenterHasNoCurvature) {
    SurfaceMesh mesh(test_meshes::fan());
    Geometry geom(mesh, test_meshes::fanPositions());
    EXPECT_NEAR(geom.angleSum(4), 2.0 * kPi, kTol);
    EXPECT_NEAR(geom.angleDefect(4), 0.0, kTol);
    EXPECT_TRUE(geom.vertexNormal(4).isApprox(Eigen::Vector3d::UnitZ()));
}

// Lifting the center makes a pyramid. Each apex angle has cosine 1/3 at height 1.
TEST(GeometryFan, PyramidApexHasPositiveCurvature) {
    SurfaceMesh mesh(test_meshes::fan());
    Geometry geom(mesh, test_meshes::fanPositions(1.0));
    EXPECT_NEAR(geom.angleDefect(4), 2.0 * kPi - 4.0 * std::acos(1.0 / 3.0), kTol);
    EXPECT_GT(geom.angleDefect(4), 0.0);
    EXPECT_TRUE(geom.vertexNormal(4).isApprox(Eigen::Vector3d::UnitZ()));
}

// -- Construction and updates ----------------------------------------------

TEST(GeometryInvalid, RejectsTooFewPositions) {
    SurfaceMesh mesh(test_meshes::singleTriangle());
    Positions positions = {{0, 0, 0}, {1, 0, 0}};
    EXPECT_THROW({ Geometry geom(mesh, positions); }, std::invalid_argument);
}

TEST(GeometryInvalid, RejectsTooManyPositions) {
    SurfaceMesh mesh(test_meshes::singleTriangle());
    Positions positions = test_meshes::singleTrianglePositions();
    positions.emplace_back(1, 1, 1);
    EXPECT_THROW({ Geometry geom(mesh, positions); }, std::invalid_argument);
}

TEST(GeometryInvalid, SetPositionsRejectsWrongSize) {
    SurfaceMesh mesh(test_meshes::singleTriangle());
    Geometry geom(mesh, test_meshes::singleTrianglePositions());
    EXPECT_THROW(geom.setPositions({{0, 0, 0}}), std::invalid_argument);
}

TEST(GeometryUpdate, SetPositionsChangesGeometry) {
    SurfaceMesh mesh(test_meshes::singleTriangle());
    Geometry geom(mesh, test_meshes::singleTrianglePositions());
    geom.setPositions({{0, 0, 0}, {2, 0, 0}, {0, 2, 0}});
    EXPECT_NEAR(geom.totalArea(), 2.0, kTol);
    EXPECT_TRUE(geom.position(1).isApprox(Eigen::Vector3d(2, 0, 0)));
}
