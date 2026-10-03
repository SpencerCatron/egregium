#pragma once
#include <egregium/mesh/surface_mesh.h>
#include <gtest/gtest.h>

using egregium::SurfaceMesh;

TEST(SurfaceMesj, TwinOfTwinIsSelf){
    SurfaceMesh mesh({{0, 1, 2}, {0, 3, 1}, {0, 2, 3}, {1, 3, 2}}); 
    for (int h = 0; h < mesh.nHalfedges(); ++h) {
        EXPECT_EQ(mesh.twin(mesh.twin(h)), h);
    }
}