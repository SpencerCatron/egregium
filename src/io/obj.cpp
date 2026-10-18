#include <egregium/io/obj.h>

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace egregium {
namespace {
// f values are v/vt/vn , we only need the v
int getVertexIndex(const std::string& s, int positionsSize) {
    int rawIndex = std::stoi(s.substr(0, s.find('/')));
    if (rawIndex == 0) {
        throw std::invalid_argument("readOBJ: 0 is not valid OBJ index");
    }
    if (rawIndex < 0) {
        return positionsSize + rawIndex;
    } else {
        return rawIndex - 1;
    }
}

// Fan triangulates a polygon and appends these triangles to given vector of faces.
// This assumes some sort of convexity on the polygon.
void appendFanTriangulation(const std::vector<int>& vertices, Faces& faces) {
    if (vertices.size() < 3) {
        throw std::invalid_argument("readObj: not enough vertices on face");
    }
    for (size_t i = 1; i < vertices.size() - 1; ++i) {
        faces.push_back({vertices[0], vertices[i], vertices[i + 1]});
    }
}
}  // namespace

// TODO: add handing for unused indices, add other forms of triangulation.
MeshData readObj(const std::filesystem::path& path) {
    Faces faces;
    std::vector<Eigen::Vector3d> positions;
    std::ifstream file(path);
    if (!file) {
        throw std::runtime_error("readObj cannot open " + path.string());
    }
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream ss(line);
        std::string type;

        ss >> type;

        if (type == "v") {
            double x, y, z;
            ss >> x >> y >> z;
            if (!ss) {
                throw std::invalid_argument("readObj: invalid vertex");
            }
            positions.push_back(Eigen::Vector3d(x, y, z));
        }
        else if (type == "f") {
            std::vector<int> vertices;
            std::string rawVertexData;
            while (ss >> rawVertexData) {
                vertices.push_back( getVertexIndex(rawVertexData, static_cast<int>(positions.size())));
            }
            appendFanTriangulation(vertices, faces);
        }
    }

    return MeshData(std::move(faces), std::move(positions));
}

}  // namespace egregium