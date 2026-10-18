#pragma once
#include <egregium/mesh/surface_mesh.h>

#include <Eigen/Dense>
#include <filesystem>
#include <vector>

namespace egregium {

struct MeshData {
    Faces faces;
    std::vector<Eigen::Vector3d> positions;
};

MeshData readObj(const std::filesystem::path& path);

}  // namespace egregium