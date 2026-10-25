#include <egregium/geometry/geometry.h>
#include <egregium/io/obj.h>
#include <egregium/mesh/surface_mesh.h>
#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>

#include "test_meshes.h"

using egregium::Faces;
using egregium::Geometry;
using egregium::MeshData;
using egregium::readObj;
using egregium::SurfaceMesh;
using egregium::writeObj;
using test_meshes::Positions;

namespace {

// writeObj takes an n x 3 matrix; the test meshes provide one Vector3d per vertex.
Eigen::MatrixXd toMatrix(const Positions& positions) {
    Eigen::MatrixXd P(positions.size(), 3);
    for (int i = 0; i < static_cast<int>(positions.size()); ++i) {
        P.row(i) = positions[i].transpose();
    }
    return P;
}

// Gives each test its own temporary OBJ file and deletes it afterwards.
class ObjTest : public ::testing::Test {
 protected:
    void SetUp() override {
        const std::string name = ::testing::UnitTest::GetInstance()->current_test_info()->name();
        path_ = std::filesystem::temp_directory_path() / ("egregium_test_" + name + ".obj");
    }

    void TearDown() override {
        std::filesystem::remove(path_);
    }

    // Writes the given text to the temporary file and returns its path.
    const std::filesystem::path& writeText(const std::string& text) {
        std::ofstream file(path_);
        file << text;
        return path_;
    }

    // Reads the temporary file back as a single string.
    std::string readText() const {
        std::ifstream file(path_);
        return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
    }

    std::filesystem::path path_;
};

}  // namespace

// -- Round trips -------------------------------------------------------------

// Torus positions come from sin and cos, so they have full double precision. Exact
// equality only holds if writeObj prints enough digits.
TEST_F(ObjTest, RoundTripIsExact) {
    const Faces faces = test_meshes::torus();
    const Eigen::MatrixXd positions = toMatrix(test_meshes::torusPositions());

    writeObj(path_, faces, positions);
    MeshData data = readObj(path_);

    EXPECT_EQ(data.faces, faces);
    ASSERT_EQ(data.positions.rows(), positions.rows());
    ASSERT_EQ(data.positions.cols(), 3);
    for (int i = 0; i < positions.rows(); ++i) {
        const Eigen::Vector3d expected = positions.row(i).transpose();
        const Eigen::Vector3d actual = data.positions.row(i).transpose();
        EXPECT_EQ(actual, expected) << "vertex " << i;
    }
}

// OBJ indices start at 1. A round trip alone would miss a writer and reader that are
// off by one in the same way, so check the text directly.
TEST_F(ObjTest, WritesOneBasedIndices) {
    writeObj(path_, test_meshes::singleTriangle(),
             toMatrix(test_meshes::singleTrianglePositions()));
    EXPECT_EQ(readText(), "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n");
}

// A loaded file should be usable to build a mesh and its geometry.
TEST_F(ObjTest, LoadedMeshBuildsGeometry) {
    writeObj(path_, test_meshes::octahedron(), toMatrix(test_meshes::octahedronPositions()));
    MeshData data = readObj(path_);

    SurfaceMesh mesh(data.faces);
    Geometry geom(mesh, std::move(data.positions));
    EXPECT_EQ(mesh.nVertices(), 6);
    EXPECT_EQ(mesh.nFaces(), 8);
    EXPECT_NEAR(geom.totalArea(), 4.0 * std::sqrt(3.0), 1e-12);
}

// -- Parsing -----------------------------------------------------------------

TEST_F(ObjTest, ReadsAllFaceEntryForms) {
    const auto& path = writeText(
        "# unit square with normals and texture coordinates\n"
        "v 0 0 0\n"
        "v 1 0 0\n"
        "v 1 1 0\n"
        "v 0 1 0\n"
        "vt 0 0\n"
        "vn 0 0 1\n"
        "\n"
        "f 1/1/1 2/1/1 3/1/1\n"
        "f 1//1 3//1 4\n");
    MeshData data = readObj(path);

    const Faces expected = {{0, 1, 2}, {0, 2, 3}};
    EXPECT_EQ(data.faces, expected);
    ASSERT_EQ(data.positions.rows(), 4);
    const Eigen::Vector3d p2 = data.positions.row(2).transpose();
    EXPECT_EQ(p2, Eigen::Vector3d(1, 1, 0));
}

// Negative indices count back from the last vertex read so far.
TEST_F(ObjTest, ReadsNegativeIndices) {
    const auto& path = writeText(
        "v 0 0 0\n"
        "v 1 0 0\n"
        "v 0 1 0\n"
        "f -3 -2 -1\n");
    MeshData data = readObj(path);

    const Faces expected = {{0, 1, 2}};
    EXPECT_EQ(data.faces, expected);
}

TEST_F(ObjTest, FanTriangulatesPolygons) {
    const auto& path = writeText(
        "v 0 0 0\n"
        "v 1 0 0\n"
        "v 2 1 0\n"
        "v 1 2 0\n"
        "v 0 1 0\n"
        "f 1 2 3 4 5\n");
    MeshData data = readObj(path);

    const Faces expected = {{0, 1, 2}, {0, 2, 3}, {0, 3, 4}};
    EXPECT_EQ(data.faces, expected);
}

// -- Errors ------------------------------------------------------------------

TEST_F(ObjTest, RejectsMissingFile) {
    EXPECT_THROW(readObj(path_), std::runtime_error);  // never written, so it doesn't exist
}

TEST_F(ObjTest, RejectsUnwritablePath) {
    const auto path = path_.parent_path() / "egregium_no_such_dir" / "mesh.obj";
    const Eigen::MatrixXd positions = toMatrix(test_meshes::singleTrianglePositions());
    EXPECT_THROW(writeObj(path, test_meshes::singleTriangle(), positions), std::runtime_error);
}

TEST_F(ObjTest, RejectsZeroIndex) {
    const auto& path = writeText("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 0 1 2\n");
    EXPECT_THROW(readObj(path), std::runtime_error);
}

TEST_F(ObjTest, RejectsShortVertexLine) {
    const auto& path = writeText("v 0 0\n");
    EXPECT_THROW(readObj(path), std::runtime_error);
}

TEST_F(ObjTest, RejectsFaceWithTwoVertices) {
    const auto& path = writeText("v 0 0 0\nv 1 0 0\nf 1 2\n");
    EXPECT_THROW(readObj(path), std::runtime_error);
}

// The header promises std::runtime_error for malformed files, including bad indices.
TEST_F(ObjTest, RejectsNonNumericIndex) {
    const auto& path = writeText("v 0 0 0\nv 1 0 0\nv 0 1 0\nf a b c\n");
    EXPECT_THROW(readObj(path), std::runtime_error);
}
