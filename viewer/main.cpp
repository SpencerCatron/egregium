#include <egregium/geometry/geometry.h>
#include <egregium/io/obj.h>
#include <egregium/mesh/surface_mesh.h>
#include <polyscope/polyscope.h>
#include <polyscope/surface_mesh.h>

#include <filesystem>

#include "imgui.h"

int main() {
    const std::filesystem::path meshPath =
        std::filesystem::path(EGREGIUM_DATA_DIR) / "spot_triangulated.obj";
    auto data = egregium::readObj(meshPath);
    egregium::SurfaceMesh mesh(data.faces);
    egregium::Geometry geom(mesh, data.positions);

    std::vector<double> gaussian(mesh.nVertices());
    std::vector<Eigen::Vector3d> faceNormal(mesh.nFaces());

    for (int v = 0; v < mesh.nVertices(); ++v) {
        gaussian[v] = geom.angleDefect(v) / geom.vertexDualArea(v);
    }
    for (int f = 0; f < mesh.nFaces(); ++f) {
        faceNormal[f] = geom.faceNormal(f);
    }

    polyscope::init();
    auto* psMesh = polyscope::registerSurfaceMesh("Spot", data.positions, data.faces);
    psMesh->addVertexScalarQuantity("Guassian curvature", gaussian);
    psMesh->addFaceVectorQuantity("Face Normal", faceNormal);
    polyscope::show();
}