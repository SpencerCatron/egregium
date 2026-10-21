#include <egregium/io/obj.h>

#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace egregium {
namespace {
// Face entries are v, v/vt, v//vn, and v/vt/vn; only v is used.
int getVertexIndex(const std::string& s, int nPositions) {
    int rawIndex = std::stoi(s.substr(0, s.find('/')));
    if (rawIndex == 0) {
        throw std::runtime_error("readObj: 0 is not valid OBJ index");
    }
    if (rawIndex < 0) {
        return nPositions + rawIndex;
    } else {
        return rawIndex - 1;
    }
}

// Fan triangulates a polygon and appends these triangles to given vector of faces.
// This assumes some sort of convexity on the polygon.
void appendFanTriangulation(const std::vector<int>& vertices, Faces& faces) {
    if (vertices.size() < 3) {
        throw std::runtime_error("readObj: not enough vertices on face");
    }
    for (std::size_t i = 1; i < vertices.size() - 1; ++i) {
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
                throw std::runtime_error("readObj: invalid vertex");
            }
            positions.push_back(Eigen::Vector3d(x, y, z));
        } else if (type == "f") {
            std::vector<int> vertices;
            std::string rawVertexData;
            while (ss >> rawVertexData) {
                vertices.push_back(
                    getVertexIndex(rawVertexData, static_cast<int>(positions.size())));
            }
            appendFanTriangulation(vertices, faces);
        }
    }

    return MeshData(std::move(faces), std::move(positions));
}

void writeObj(const std::filesystem::path& path, const Faces& faces,
              const std::vector<Eigen::Vector3d>& positions) {
    std::ofstream file(path);
    if (!file) {
        throw std::runtime_error("writeOBJ: failed while writing " + path.string());
    }

    file.precision(std::numeric_limits<double>::max_digits10);
    for (const auto& p : positions) {
        file << "v " << p.x() << ' ' << p.y() << ' ' << p.z() << '\n';
    }
    for (const auto& f : faces) {
        file << "f " << f[0] + 1 << ' ' << f[1] + 1 << ' ' << f[2] + 1 << '\n';
    }
}
}  // namespace egregium