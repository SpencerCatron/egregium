#pragma once
#include <egregium/mesh/surface_mesh.h>

#include <Eigen/Dense>
#include <filesystem>
#include <vector>

namespace egregium {

// Container for reading and writing data.
struct MeshData {
    Faces faces;
    Eigen::MatrixXd positions;
};

// Reads vertex positions and faces from OBJ file.
// Polygons are fan triangulated.
// Throws std::runtime_error if file cannot be opened, is malformed.
MeshData readObj(const std::filesystem::path& path);

// Writes positions and triangular faces as OBJ file.
// Throws std::runtime_error if file cannot be opened, or cannot be written.
void writeObj(const std::filesystem::path& path, const Faces& faces,
              const Eigen::MatrixXd& positions);
}  // namespace egregium