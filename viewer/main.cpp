#include <egregium/algorithms/heat_diffusion.h>
#include <egregium/geometry/geometry.h>
#include <egregium/io/obj.h>
#include <egregium/mesh/surface_mesh.h>
#include <polyscope/polyscope.h>
#include <polyscope/surface_mesh.h>
#include<egregium/algorithms/mean_curvature_flow.h>

#include <filesystem>
#include <vector>

int main() {
    const std::filesystem::path meshPath =
        std::filesystem::path(EGREGIUM_DATA_DIR) / "spot_triangulated.obj";
    auto data = egregium::readObj(meshPath);
    egregium::SurfaceMesh mesh(data.faces);
    egregium::Geometry geom(mesh, data.positions);

    std::vector<double> gaussian(mesh.nVertices());
    std::vector<double> meanCurvature(mesh.nVertices());
    std::vector<Eigen::Vector3d> faceNormal(mesh.nFaces());

    for (int v = 0; v < mesh.nVertices(); ++v) {
        gaussian[v] = geom.gaussianCurvature(v);
        meanCurvature[v] = geom.meanCurvature(v);
    }
    for (int f = 0; f < mesh.nFaces(); ++f) {
        faceNormal[f] = geom.faceNormal(f);
    }


    egregium::HeatDiffusion heat(geom);
    egregium::MeanCurvatureFlow flow(geom.mesh(), geom.positionMatrix());



    Eigen::VectorXd u = Eigen::VectorXd::Zero(mesh.nVertices());
    u(0) = 10.0;
    heat.setState(u);

    polyscope::init();
    auto* psMesh = polyscope::registerSurfaceMesh("Spot", data.positions, data.faces);
    psMesh->addVertexScalarQuantity("Gaussian curvature", gaussian);
    psMesh->addVertexScalarQuantity("Mean curvature", meanCurvature);
    psMesh->addFaceVectorQuantity("Face Normal", faceNormal);
    psMesh->addVertexScalarQuantity("temperature", heat.state());

    bool flag1 = false;
    bool flag2 = false;

    polyscope::state::userCallback = [&]() {
        ImGui::SeparatorText("Heat demo");
        if (ImGui::Button("Start##heat")) {
            flag1 = true;
        }
        if (flag1){
            heat.step();
            psMesh->addVertexScalarQuantity("temperature", heat.state());
        }
        if(ImGui::Button("Reset##heat")){
            flag1=false;
            heat.setState(u);
            psMesh->addVertexScalarQuantity("temperature", heat.state());
        }
        ImGui::SeparatorText("Mean curvature flow demo");
        if (ImGui::Button("Start##mcf")) {
            flag2 = true;
        }
        if(flag2){
            flow.step();
            psMesh->updateVertexPositions(flow.state());
        }
        if(ImGui::Button("Reset##mcf")){
            flag2=false;
            psMesh->updateVertexPositions(geom.positionMatrix());
        }
    };

    polyscope::show();
}
