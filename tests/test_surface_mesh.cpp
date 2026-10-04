#pragma once
#include <egregium/mesh/surface_mesh.h>
#include <egregium/tests/test_meshes.h>
#include <gtest/gtest.h>
#include <string>
#include <ostream>



using egregium::SurfaceMesh;

//testing class. mesh structure with added information
struct MeshCase{
    std::string name;
    test_meshes::Faces faces;
    int v, e, f, chi;
};
void PrintTo(const MeshCase& c, std::ostream* os) { *os << c.name; }

class ValidMeshTest : public ::testing::TestWithParam<MeshCase>{};

TEST_P(ValidMeshTest, counts ){
    const MeshCase& c = GetParam();
    SurfaceMesh mesh(c.faces);
    EXPECT_EQ(mesh.nVertices(), c.v);
    EXPECT_EQ(mesh.nEdges(), c.e);
    EXPECT_EQ(mesh.nFaces(), c.f); 
}
TEST_P(ValidMeshTest, eulerCharcteristic ){
    const MeshCase& c = GetParam();
    SurfaceMesh mesh(c.faces);
    EXPECT_EQ(mesh.eulerCharaceristic(), c.chi);
}
TEST_P(ValidMeshTest, TwinOfTwinIsSelf) {
    SurfaceMesh mesh(GetParam().faces);
    for (int h = 0; h < mesh.nHalfedges(); ++h) {
        EXPECT_EQ(mesh.twin(mesh.twin(h)), h) << "half-edge " << h;
    }
}
TEST_P(ValidMeshTest, NextNextNextIsSelf){
    SurfaceMesh mesh(GetParam().faces);
    for (int h = 0; h < mesh.nHalfedges(); ++h){
        if(!mesh.isBoundaryHalfedge(h)){
            EXPECT_EQ(mesh.next(mesh.next(mesh.next(h))), h) << "half-edge " << h;
        }
    }
} 
TEST_P(ValidMeshTest, TwinIsNotSelf) {
    SurfaceMesh mesh(GetParam().faces);
    for (int h = 0; h < mesh.nHalfedges(); ++h) {
        EXPECT_NE(mesh.twin(h), h) << "half-edge " << h;
    }
} 
TEST_P(ValidMeshTest, TwinTailIsSelfHead) {
    SurfaceMesh mesh(GetParam().faces);
    for (int h = 0; h < mesh.nHalfedges(); ++h) {
        EXPECT_EQ(mesh.tailVertex(mesh.twin(h)), mesh.tailVertex(mesh.next(h))) << "half-edge " << h;
    }
}

INSTANTIATE_TEST_SUITE_P(
    AllValidMeshes, ValidMeshTest,
    ::testing::Values(
        MeshCase{"tetrahedron", test_meshes::tetrahedron(), 4, 6, 4 ,2},
        MeshCase{"octahedron",  test_meshes::octahedron(),  6, 12, 8, 2},
        MeshCase{"cube",        test_meshes::cube(),        8, 18, 12 ,2},
        MeshCase{"torus",       test_meshes::torus(),       9, 27, 18, 0},
        MeshCase{"triangle",    test_meshes::singleTriangle(), 3, 3, 1 ,1},
        MeshCase{"square",      test_meshes::square(),      4, 5, 2 ,1},
        MeshCase{"fan",         test_meshes::fan(),         5, 8, 4 ,1},
        MeshCase{"annulus",     test_meshes::annulus(),     8, 16, 8, 0}),
    [](const ::testing::TestParamInfo<MeshCase>& info) { return info.param.name; });
