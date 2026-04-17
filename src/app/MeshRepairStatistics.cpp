#include "MeshRepairStatistics.h"

#include "StlHoleFilling.h"

#include <CGAL/Polygon_mesh_processing/self_intersections.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace PMP = CGAL::Polygon_mesh_processing;

namespace Mayo {

    namespace {

        struct Tri3 {
            double x[3];
            double y[3];
            double z[3];
        };

        using QuantKey = std::tuple<long long, long long, long long>;
        struct QuantKeyHash {
            std::size_t operator()(const QuantKey& k) const noexcept {
                std::size_t h1 = std::hash<long long>{}(std::get<0>(k));
                std::size_t h2 = std::hash<long long>{}(std::get<1>(k));
                std::size_t h3 = std::hash<long long>{}(std::get<2>(k));
                h1 ^= h2 + 0x9e3779b9 + (h1 << 6) + (h1 >> 2);
                h1 ^= h3 + 0x9e3779b9 + (h1 << 6) + (h1 >> 2);
                return h1;
            }
        };
        using Edge = std::pair<int, int>;

        inline std::string toLower(std::string s)
        {
            std::transform(s.begin(), s.end(), s.begin(),
                [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            return s;
        }

        inline std::string fileExt(const std::string& filepath)
        {
            const auto pos = filepath.find_last_of('.');
            if (pos == std::string::npos || pos + 1 >= filepath.size())
                return {};

            return toLower(filepath.substr(pos + 1));
        }

        inline QuantKey makeQuantKey(double x, double y, double z)
        {
            constexpr double scale = 1e6;
            return {
                static_cast<long long>(std::llround(x * scale)),
                static_cast<long long>(std::llround(y * scale)),
                static_cast<long long>(std::llround(z * scale))
            };
        }

        inline Edge makeEdge(int a, int b)
        {
            if (a > b)
                std::swap(a, b);
            return { a, b };
        }

        bool loadTrianglesFromStl(const std::string& filepath, std::vector<Tri3>& out)
        {
            std::vector<Triangles> triangles;
            if (isBinarySTL(filepath))
                readBinarySTL(filepath, triangles);
            else
                readASCIISTL(filepath, triangles);

            out.clear();
            out.reserve(triangles.size());
            for (const Triangles& tri : triangles) {
                Tri3 t{};
                t.x[0] = tri.v1.x; t.y[0] = tri.v1.y; t.z[0] = tri.v1.z;
                t.x[1] = tri.v2.x; t.y[1] = tri.v2.y; t.z[1] = tri.v2.z;
                t.x[2] = tri.v3.x; t.y[2] = tri.v3.y; t.z[2] = tri.v3.z;
                out.push_back(t);
            }

            return !out.empty();
        }

        int parseObjIndex(const std::string& token, int vertexCount)
        {
            const std::size_t slashPos = token.find('/');
            const std::string indexStr = token.substr(0, slashPos);
            if (indexStr.empty())
                return -1;

            int idx = std::stoi(indexStr);
            if (idx > 0)
                return idx - 1;

            if (idx < 0)
                return vertexCount + idx;

            return -1;
        }

        bool loadTrianglesFromObj(const std::string& filepath, std::vector<Tri3>& out, std::string* error)
        {
            std::ifstream file(filepath);
            if (!file.is_open()) {
                if (error)
                    *error = "Cannot open OBJ file";
                return false;
            }

            struct V3 { double x = 0; double y = 0; double z = 0; };
            std::vector<V3> vertices;
            out.clear();

            std::string line;
            while (std::getline(file, line)) {
                std::stringstream ss(line);
                std::string tag;
                ss >> tag;

                if (tag == "v") {
                    V3 v;
                    if (ss >> v.x >> v.y >> v.z)
                        vertices.push_back(v);
                }
                else if (tag == "f") {
                    std::vector<int> faceIndices;
                    std::string token;
                    while (ss >> token) {
                        try {
                            const int idx = parseObjIndex(token, static_cast<int>(vertices.size()));
                            if (idx >= 0 && idx < static_cast<int>(vertices.size()))
                                faceIndices.push_back(idx);
                        }
                        catch (...) {
                        }
                    }

                    if (faceIndices.size() >= 3) {
                        for (std::size_t i = 1; i + 1 < faceIndices.size(); ++i) {
                            const V3& a = vertices[faceIndices[0]];
                            const V3& b = vertices[faceIndices[i]];
                            const V3& c = vertices[faceIndices[i + 1]];
                            Tri3 t{};
                            t.x[0] = a.x; t.y[0] = a.y; t.z[0] = a.z;
                            t.x[1] = b.x; t.y[1] = b.y; t.z[1] = b.z;
                            t.x[2] = c.x; t.y[2] = c.y; t.z[2] = c.z;
                            out.push_back(t);
                        }
                    }
                }
            }

            if (out.empty() && error)
                *error = "OBJ contains no triangulatable faces";

            return !out.empty();
        }

        bool loadTrianglesFromPly(const std::string& filepath, std::vector<Tri3>& out, std::string* error)
        {
            std::ifstream file(filepath);
            if (!file.is_open()) {
                if (error)
                    *error = "Cannot open PLY file";
                return false;
            }

            int numVertices = 0;
            int numFaces = 0;
            bool ascii = false;

            std::string line;
            while (std::getline(file, line)) {
                if (line.rfind("format", 0) == 0 && line.find("ascii") != std::string::npos)
                    ascii = true;
                else if (line.rfind("element vertex", 0) == 0)
                    numVertices = std::stoi(line.substr(15));
                else if (line.rfind("element face", 0) == 0)
                    numFaces = std::stoi(line.substr(13));
                else if (line == "end_header")
                    break;
            }

            if (!ascii) {
                if (error)
                    *error = "Only ASCII PLY is supported for statistics";
                return false;
            }

            struct V3 { double x = 0; double y = 0; double z = 0; };
            std::vector<V3> vertices;
            vertices.reserve(std::max(0, numVertices));

            for (int i = 0; i < numVertices; ++i) {
                V3 v;
                if (!(file >> v.x >> v.y >> v.z)) {
                    if (error)
                        *error = "Invalid vertex section in PLY";
                    return false;
                }
                vertices.push_back(v);

                std::getline(file, line); // consume tail properties
            }

            out.clear();
            for (int i = 0; i < numFaces; ++i) {
                std::getline(file >> std::ws, line);
                if (line.empty())
                    continue;

                std::stringstream ss(line);
                int n = 0;
                ss >> n;
                if (n < 3)
                    continue;

                std::vector<int> idx(n);
                for (int k = 0; k < n; ++k)
                    ss >> idx[k];

                for (int k = 1; k + 1 < n; ++k) {
                    if (idx[0] < 0 || idx[k] < 0 || idx[k + 1] < 0 ||
                        idx[0] >= static_cast<int>(vertices.size()) ||
                        idx[k] >= static_cast<int>(vertices.size()) ||
                        idx[k + 1] >= static_cast<int>(vertices.size())) {
                        continue;
                    }

                    const V3& a = vertices[idx[0]];
                    const V3& b = vertices[idx[k]];
                    const V3& c = vertices[idx[k + 1]];
                    Tri3 t{};
                    t.x[0] = a.x; t.y[0] = a.y; t.z[0] = a.z;
                    t.x[1] = b.x; t.y[1] = b.y; t.z[1] = b.z;
                    t.x[2] = c.x; t.y[2] = c.y; t.z[2] = c.z;
                    out.push_back(t);
                }
            }

            if (out.empty() && error)
                *error = "PLY contains no triangulatable faces";

            return !out.empty();
        }

        bool loadMeshTriangles(const std::string& filepath, std::vector<Tri3>& out, std::string& outFormat, std::string* error)
        {
            const std::string ext = fileExt(filepath);
            outFormat = ext;

            if (ext == "stl")
                return loadTrianglesFromStl(filepath, out);

            if (ext == "obj")
                return loadTrianglesFromObj(filepath, out, error);

            if (ext == "ply")
                return loadTrianglesFromPly(filepath, out, error);

            if (error)
                *error = "Unsupported file type for statistics. Supported: STL, OBJ, PLY";

            return false;
        }

    } // namespace

    std::optional<MeshRepairStats> computeMeshRepairStatsFromMeshFile(
        const std::string& meshFilepath,
        std::string* errorMessage)
    {
        std::vector<Tri3> triangles;
        std::string format;
        if (!loadMeshTriangles(meshFilepath, triangles, format, errorMessage) || triangles.empty()) {
            if (errorMessage && errorMessage->empty())
                *errorMessage = "No triangles were read from mesh file";
            return std::nullopt;
        }

        MeshRepairStats stats;
        stats.inputFormat = format;
        stats.triangleCount = static_cast<int>(triangles.size());

        std::unordered_map<QuantKey, int, QuantKeyHash> mapVertexIndex;
        mapVertexIndex.reserve(triangles.size() * 3);

        struct V3 { double x = 0; double y = 0; double z = 0; };
        std::vector<V3> uniqueVerts;
        std::vector<std::array<int, 3>> triIndices;
        triIndices.reserve(triangles.size());

        auto fnGetVertexIndex = [&](double x, double y, double z) {
            const QuantKey key = makeQuantKey(x, y, z);
            auto it = mapVertexIndex.find(key);
            if (it != mapVertexIndex.end())
                return it->second;

            const int idx = static_cast<int>(mapVertexIndex.size());
            mapVertexIndex.insert({ key, idx });
            uniqueVerts.push_back({ x, y, z });
            return idx;
            };

        for (const Tri3& tri : triangles) {
            const int i1 = fnGetVertexIndex(tri.x[0], tri.y[0], tri.z[0]);
            const int i2 = fnGetVertexIndex(tri.x[1], tri.y[1], tri.z[1]);
            const int i3 = fnGetVertexIndex(tri.x[2], tri.y[2], tri.z[2]);
            triIndices.push_back({ i1, i2, i3 });
        }

        stats.vertexCount = static_cast<int>(mapVertexIndex.size());
        stats.duplicateVertices = stats.triangleCount * 3 - stats.vertexCount;

        std::map<Edge, int> mapEdgeUseCount;
        for (const auto& tri : triIndices) {
            mapEdgeUseCount[makeEdge(tri[0], tri[1])] += 1;
            mapEdgeUseCount[makeEdge(tri[1], tri[2])] += 1;
            mapEdgeUseCount[makeEdge(tri[2], tri[0])] += 1;
        }

        for (const auto& pair : mapEdgeUseCount) {
            if (pair.second > 2)
                ++stats.nonManifoldEdges;
        }

        SurfaceMesh mesh;
        std::vector<SurfaceMesh::Vertex_index> smVerts(uniqueVerts.size());
        for (std::size_t i = 0; i < uniqueVerts.size(); ++i)
            smVerts[i] = mesh.add_vertex(Point(uniqueVerts[i].x, uniqueVerts[i].y, uniqueVerts[i].z));

        for (const auto& tri : triIndices) {
            mesh.add_face(smVerts[tri[0]], smVerts[tri[1]], smVerts[tri[2]]);
        }

        stats.holeCount = static_cast<int>(countHolesCGAL(mesh));

        std::vector<std::pair<SurfaceMesh::Face_index, SurfaceMesh::Face_index>> selfIntersections;
        PMP::self_intersections(mesh, std::back_inserter(selfIntersections));
        stats.selfIntersectionPairs = static_cast<int>(selfIntersections.size());

        if (!uniqueVerts.empty()) {
            constexpr int voxelRes = 64;
            stats.voxelResolution = voxelRes;

            double minX = uniqueVerts[0].x, minY = uniqueVerts[0].y, minZ = uniqueVerts[0].z;
            double maxX = minX, maxY = minY, maxZ = minZ;
            for (const auto& v : uniqueVerts) {
                minX = std::min(minX, v.x); minY = std::min(minY, v.y); minZ = std::min(minZ, v.z);
                maxX = std::max(maxX, v.x); maxY = std::max(maxY, v.y); maxZ = std::max(maxZ, v.z);
            }

            const double dx = std::max(1e-9, maxX - minX);
            const double dy = std::max(1e-9, maxY - minY);
            const double dz = std::max(1e-9, maxZ - minZ);

            auto toVoxel = [&](double x, double y, double z) {
                const int ix = std::clamp(static_cast<int>(((x - minX) / dx) * (voxelRes - 1)), 0, voxelRes - 1);
                const int iy = std::clamp(static_cast<int>(((y - minY) / dy) * (voxelRes - 1)), 0, voxelRes - 1);
                const int iz = std::clamp(static_cast<int>(((z - minZ) / dz) * (voxelRes - 1)), 0, voxelRes - 1);
                return ix + voxelRes * (iy + voxelRes * iz);
                };

            std::unordered_set<int> occupied;
            occupied.reserve(triangles.size() * 4);
            for (const Tri3& tri : triangles) {
                occupied.insert(toVoxel(tri.x[0], tri.y[0], tri.z[0]));
                occupied.insert(toVoxel(tri.x[1], tri.y[1], tri.z[1]));
                occupied.insert(toVoxel(tri.x[2], tri.y[2], tri.z[2]));
                occupied.insert(toVoxel(
                    (tri.x[0] + tri.x[1] + tri.x[2]) / 3.0,
                    (tri.y[0] + tri.y[1] + tri.y[2]) / 3.0,
                    (tri.z[0] + tri.z[1] + tri.z[2]) / 3.0));
            }

            stats.voxelOccupiedCells = static_cast<int>(occupied.size());
        }

        return stats;
    }

} // namespace Mayo