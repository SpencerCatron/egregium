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

// Reads vertex positions and faces from OBJ file.
// Polygons are fan triangulated.
// Throws std::runtime_error if file cannot be opened, is malformed, or cannot be written.
MeshData readObj(const std::filesystem::path& path);

// Writes positions and triangular faces as OBJ file.
// Throws std::runtime_error if file cannot be opened.
void writeObj(const std::filesystem::path& path, const Faces& faces,
              const std::vector<Eigen::Vector3d>& positions);
}  // namespace egregium