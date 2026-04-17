#include "WatertightMesh.h"

#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <map>
#include <set>
#include <tuple>
#include <queue>

namespace Mayo {

    // =============================================================================
    //  Internal math helpers
    // =============================================================================

    static float wtDot(const WTVec3& a, const WTVec3& b)
    {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    static WTVec3 wtCross(const WTVec3& a, const WTVec3& b)
    {
        return { a.y * b.z - a.z * b.y,
                 a.z * b.x - a.x * b.z,
                 a.x * b.y - a.y * b.x };
    }

    static WTBBox wtGetMeshBBox(const WTMesh& mesh)
    {
        WTBBox box;
        if (mesh.vertices.empty()) return box;
        box.min = box.max = { mesh.vertices[0].x, mesh.vertices[0].y, mesh.vertices[0].z };
        for (const auto& v : mesh.vertices) {
            box.min.x = std::min(box.min.x, v.x);
            box.min.y = std::min(box.min.y, v.y);
            box.min.z = std::min(box.min.z, v.z);
            box.max.x = std::max(box.max.x, v.x);
            box.max.y = std::max(box.max.y, v.y);
            box.max.z = std::max(box.max.z, v.z);
        }
        return box;
    }

    // =============================================================================
    //  WTVoxelGrid implementation
    // =============================================================================

    WTVoxelGrid::WTVoxelGrid(int r, const WTBBox& box) : res(r)
    {
        data.assign(static_cast<size_t>(res) * res * res, 0);

        WTVec3 span = { box.max.x - box.min.x,
                        box.max.y - box.min.y,
                        box.max.z - box.min.z };
        // 5% padding so surface triangles never touch the grid boundary
        minBound = { box.min.x - span.x * 0.05f,
                     box.min.y - span.y * 0.05f,
                     box.min.z - span.z * 0.05f };
        maxBound = { box.max.x + span.x * 0.05f,
                     box.max.y + span.y * 0.05f,
                     box.max.z + span.z * 0.05f };
        scale = { static_cast<float>(res) / (maxBound.x - minBound.x),
                  static_cast<float>(res) / (maxBound.y - minBound.y),
                  static_cast<float>(res) / (maxBound.z - minBound.z) };
    }

    void WTVoxelGrid::set(int x, int y, int z, uint8_t val)
    {
        if (x >= 0 && x < res && y >= 0 && y < res && z >= 0 && z < res)
            data[static_cast<size_t>(x) * res * res + y * res + z] = val;
    }

    uint8_t WTVoxelGrid::get(int x, int y, int z) const
    {
        if (x >= 0 && x < res && y >= 0 && y < res && z >= 0 && z < res)
            return data[static_cast<size_t>(x) * res * res + y * res + z];
        return 2; // out-of-bounds = outside
    }

    WTVec3 WTVoxelGrid::gridToWorld(int x, int y, int z) const
    {
        return { minBound.x + static_cast<float>(x) / scale.x,
                 minBound.y + static_cast<float>(y) / scale.y,
                 minBound.z + static_cast<float>(z) / scale.z };
    }

    // =============================================================================
    //  Mesh loaders
    // =============================================================================

    bool wtLoadOBJ(const std::string& path, WTMesh& mesh)
    {
        std::ifstream file(path);
        if (!file.is_open()) return false;

        mesh.vertices.clear();
        mesh.faces.clear();
        std::string line;

        while (std::getline(file, line)) {
            std::istringstream ss(line);
            std::string type;
            ss >> type;

            if (type == "v") {
                WTVertex v;
                ss >> v.x >> v.y >> v.z;
                mesh.vertices.push_back(v);
            }
            else if (type == "f") {
                WTFace f;
                std::string seg;
                while (ss >> seg) {
                    size_t slash = seg.find('/');
                    try {
                        int idx = std::stoi(seg.substr(0, slash)) - 1;
                        f.vertexIndices.push_back(idx);
                    }
                    catch (...) {}
                }
                if (!f.vertexIndices.empty())
                    mesh.faces.push_back(f);
            }
        }
        return !mesh.vertices.empty();
    }

    bool wtLoadSTL(const std::string& path, WTMesh& mesh)
    {
        std::ifstream file(path, std::ios::binary);
        if (!file.is_open()) return false;

        mesh.vertices.clear();
        mesh.faces.clear();

        char header[6] = {};
        file.read(header, 5);
        bool isASCII = (std::strncmp(header, "solid", 5) == 0);
        file.seekg(0);

        if (isASCII) {
            std::string line;
            while (std::getline(file, line)) {
                std::istringstream ss(line);
                std::string word;
                ss >> word;
                if (word == "vertex") {
                    WTVertex v;
                    ss >> v.x >> v.y >> v.z;
                    mesh.vertices.push_back(v);
                    if (mesh.vertices.size() % 3 == 0) {
                        int last = static_cast<int>(mesh.vertices.size()) - 1;
                        mesh.faces.push_back({ {last - 2, last - 1, last} });
                    }
                }
            }
        }
        else {
            file.seekg(80);
            uint32_t numFaces = 0;
            file.read(reinterpret_cast<char*>(&numFaces), 4);

            for (uint32_t i = 0; i < numFaces; ++i) {
                float n[3];
                file.read(reinterpret_cast<char*>(n), 12);

                for (int k = 0; k < 3; ++k) {
                    WTVertex v;
                    file.read(reinterpret_cast<char*>(&v.x), 4);
                    file.read(reinterpret_cast<char*>(&v.y), 4);
                    file.read(reinterpret_cast<char*>(&v.z), 4);
                    mesh.vertices.push_back(v);
                }
                uint16_t attr;
                file.read(reinterpret_cast<char*>(&attr), 2);

                int last = static_cast<int>(mesh.vertices.size()) - 1;
                mesh.faces.push_back({ {last - 2, last - 1, last} });
            }
        }
        return !mesh.vertices.empty();
    }

    bool wtLoadPLY(const std::string& path, WTMesh& mesh)
    {
        std::ifstream file(path);
        if (!file.is_open()) return false;

        mesh.vertices.clear();
        mesh.faces.clear();

        int numVerts = 0, numFaces = 0;
        std::string line;
        while (std::getline(file, line)) {
            if (line.find("element vertex") != std::string::npos)
                numVerts = std::stoi(line.substr(15));
            else if (line.find("element face") != std::string::npos)
                numFaces = std::stoi(line.substr(13));
            else if (line == "end_header")
                break;
        }

        for (int i = 0; i < numVerts; ++i) {
            WTVertex v;
            file >> v.x >> v.y >> v.z;
            mesh.vertices.push_back(v);
        }
        for (int i = 0; i < numFaces; ++i) {
            int cnt;
            file >> cnt;
            WTFace f;
            for (int j = 0; j < cnt; ++j) {
                int idx; file >> idx;
                f.vertexIndices.push_back(idx);
            }
            mesh.faces.push_back(f);
        }
        return !mesh.vertices.empty();
    }

    bool wtLoadMesh(const std::string& path, WTMesh& mesh)
    {
        auto ext = path.substr(path.find_last_of('.') + 1);
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
        if (ext == "stl") return wtLoadSTL(path, mesh);
        if (ext == "obj") return wtLoadOBJ(path, mesh);
        if (ext == "ply") return wtLoadPLY(path, mesh);
        return false;
    }

    // =============================================================================
    //  Mesh savers
    // =============================================================================

    bool wtSaveOBJ(const std::string& path, const WTMesh& mesh)
    {
        std::ofstream file(path);
        if (!file.is_open()) return false;

        for (const auto& v : mesh.vertices)
            file << "v " << v.x << ' ' << v.y << ' ' << v.z << '\n';

        for (const auto& f : mesh.faces) {
            file << 'f';
            for (int idx : f.vertexIndices)
                file << ' ' << (idx + 1);
            file << '\n';
        }
        return true;
    }

    bool wtSaveSTL(const std::string& path, const WTMesh& mesh)
    {
        std::ofstream file(path, std::ios::binary);
        if (!file.is_open()) return false;

        char header[80] = {};
        std::strncpy(header, "Watertight Mesh by Koshika", sizeof(header) - 1);
        file.write(header, 80);

        uint32_t triCount = 0;
        for (const auto& f : mesh.faces)
            triCount += static_cast<uint32_t>(f.vertexIndices.size() >= 4 ? 2 : 1);
        file.write(reinterpret_cast<const char*>(&triCount), 4);

        auto writeVec = [&](float x, float y, float z) {
            file.write(reinterpret_cast<const char*>(&x), 4);
            file.write(reinterpret_cast<const char*>(&y), 4);
            file.write(reinterpret_cast<const char*>(&z), 4);
            };
        const uint16_t attr = 0;

        for (const auto& f : mesh.faces) {
            if (f.vertexIndices.size() < 3) continue;

            auto writeTri = [&](int i0, int i1, int i2) {
                const WTVertex& va = mesh.vertices[i0];
                const WTVertex& vb = mesh.vertices[i1];
                const WTVertex& vc = mesh.vertices[i2];
                WTVec3 e1 = { vb.x - va.x, vb.y - va.y, vb.z - va.z };
                WTVec3 e2 = { vc.x - va.x, vc.y - va.y, vc.z - va.z };
                WTVec3 n = wtCross(e1, e2);
                float  len = std::sqrt(wtDot(n, n));
                if (len > 1e-8f) { n.x /= len; n.y /= len; n.z /= len; }
                writeVec(n.x, n.y, n.z);
                writeVec(va.x, va.y, va.z);
                writeVec(vb.x, vb.y, vb.z);
                writeVec(vc.x, vc.y, vc.z);
                file.write(reinterpret_cast<const char*>(&attr), 2);
                };

            writeTri(f.vertexIndices[0], f.vertexIndices[1], f.vertexIndices[2]);
            if (f.vertexIndices.size() >= 4)
                writeTri(f.vertexIndices[0], f.vertexIndices[2], f.vertexIndices[3]);
        }
        return true;
    }

    bool wtSavePLY(const std::string& path, const WTMesh& mesh)
    {
        std::ofstream file(path);
        if (!file.is_open()) return false;

        file << "ply\nformat ascii 1.0\n";
        file << "element vertex " << mesh.vertices.size() << "\n";
        file << "property float x\nproperty float y\nproperty float z\n";
        file << "element face " << mesh.faces.size() << "\n";
        file << "property list uchar int vertex_indices\nend_header\n";

        for (const auto& v : mesh.vertices)
            file << v.x << " " << v.y << " " << v.z << "\n";
        for (const auto& f : mesh.faces) {
            file << f.vertexIndices.size();
            for (int idx : f.vertexIndices) file << " " << idx;
            file << "\n";
        }
        return true;
    }

    bool wtSaveMesh(const std::string& path, const WTMesh& mesh)
    {
        auto ext = path.substr(path.find_last_of('.') + 1);
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
        if (ext == "stl") return wtSaveSTL(path, mesh);
        if (ext == "ply") return wtSavePLY(path, mesh);
        return wtSaveOBJ(path, mesh);
    }

    // =============================================================================
    //  STAGE 1 — Surface hole filling (FIX 4: robust loop chaining)
    // =============================================================================

    void wtFillHolesSurface(WTMesh& mesh)
    {
        std::map<WTEdge, int> edgeCount;
        std::map<std::pair<int, int>, bool> directedEdge;

        for (const auto& f : mesh.faces) {
            const int n = static_cast<int>(f.vertexIndices.size());
            for (int i = 0; i < n; ++i) {
                int a = f.vertexIndices[i];
                int b = f.vertexIndices[(i + 1) % n];
                edgeCount[WTEdge(a, b)]++;
                directedEdge[{a, b}] = true;
            }
        }

        std::map<int, std::vector<int>> adj;
        int boundaryCount = 0;
        for (const auto& kv : edgeCount) {
            if (kv.second == 1) {
                const WTEdge& e = kv.first;
                if (directedEdge.count({ e.v[0], e.v[1] }))
                    adj[e.v[0]].push_back(e.v[1]);
                else
                    adj[e.v[1]].push_back(e.v[0]);
                ++boundaryCount;
            }
        }

        if (boundaryCount == 0)
            return;

        std::set<int> globalVisited;
        for (const auto& kv : adj) {
            int vStart = kv.first;
            if (globalVisited.count(vStart)) continue;

            std::vector<int> loop;
            std::set<int> loopVisited;
            int curr = vStart;
            bool closed = false;

            while (true) {
                loopVisited.insert(curr);
                globalVisited.insert(curr);
                loop.push_back(curr);

                // FIX 4: Find first UNVISITED neighbour instead of always [0]
                int next = -1;
                auto it = adj.find(curr);
                if (it != adj.end() && !it->second.empty()) {
                    for (int nb : it->second) {
                        if (nb == vStart && loop.size() >= 3) {
                            closed = true;
                            next = -1;
                            break;
                        }
                        if (!loopVisited.count(nb)) {
                            next = nb;
                            break;
                        }
                    }
                }
                if (closed || next == -1) break;
                curr = next;
            }

            if (!closed || loop.size() < 3) continue;

            WTVec3 c = { 0.f, 0.f, 0.f };
            for (int idx : loop) {
                c.x += mesh.vertices[idx].x;
                c.y += mesh.vertices[idx].y;
                c.z += mesh.vertices[idx].z;
            }
            float inv = 1.0f / static_cast<float>(loop.size());
            c.x *= inv; c.y *= inv; c.z *= inv;

            int cIdx = static_cast<int>(mesh.vertices.size());
            mesh.vertices.push_back({ c.x, c.y, c.z });

            for (size_t i = 0; i < loop.size(); ++i) {
                WTFace f;
                f.vertexIndices = { loop[i],
                                    loop[(i + 1) % loop.size()],
                                    cIdx };
                mesh.faces.push_back(f);
            }
        }
    }

    // =============================================================================
    //  STAGE 2a — Voxel rasterisation
    // =============================================================================

    void wtRasterizeMesh(const WTMesh& mesh, WTVoxelGrid& grid)
    {
        for (const auto& f : mesh.faces) {
            if (f.vertexIndices.size() < 3) continue;

            const WTVertex& va = mesh.vertices[f.vertexIndices[0]];
            const WTVertex& vb = mesh.vertices[f.vertexIndices[1]];
            const WTVertex& vc = mesh.vertices[f.vertexIndices[2]];

            WTVec3 v0{ va.x, va.y, va.z };
            WTVec3 v1{ vb.x, vb.y, vb.z };
            WTVec3 v2{ vc.x, vc.y, vc.z };

            float d1x = v1.x - v0.x, d1y = v1.y - v0.y, d1z = v1.z - v0.z;
            float d2x = v2.x - v0.x, d2y = v2.y - v0.y, d2z = v2.z - v0.z;
            float edgeMax = 0.f;
            for (float c : {std::abs(d1x), std::abs(d1y), std::abs(d1z),
                std::abs(d2x), std::abs(d2y), std::abs(d2z)})
                edgeMax = std::max(edgeMax, c);

            float maxScale = std::max({ grid.scale.x, grid.scale.y, grid.scale.z });
            int steps = std::min(static_cast<int>(edgeMax * maxScale) + 2, 100);

            for (int si = 0; si <= steps; ++si) {
                for (int sj = 0; sj <= steps - si; ++sj) {
                    float u = static_cast<float>(si) / steps;
                    float v = static_cast<float>(sj) / steps;
                    WTVec3 p = v0 * (1.f - u - v) + v1 * u + v2 * v;

                    int gx = static_cast<int>((p.x - grid.minBound.x) * grid.scale.x);
                    int gy = static_cast<int>((p.y - grid.minBound.y) * grid.scale.y);
                    int gz = static_cast<int>((p.z - grid.minBound.z) * grid.scale.z);

                    for (int dx = 0; dx <= 1; ++dx)
                        for (int dy = 0; dy <= 1; ++dy)
                            for (int dz = 0; dz <= 1; ++dz)
                                grid.set(gx + dx, gy + dy, gz + dz, 1);
                }
            }
        }
    }

    // =============================================================================
    //  STAGE 2b — Flood fill exterior (FIX 1: seed from ALL 6 boundary faces)
    // =============================================================================

    void wtFloodFillOutside(WTVoxelGrid& grid)
    {
        struct Node { int x, y, z; };
        std::queue<Node> q;

        auto trySeed = [&](int x, int y, int z) {
            if (grid.get(x, y, z) == 0) {
                grid.set(x, y, z, 2);
                q.push({ x, y, z });
            }
            };

        // FIX 1: Seed from ALL 6 faces of the grid
        int R = grid.res;
        for (int a = 0; a < R; ++a) {
            for (int b = 0; b < R; ++b) {
                trySeed(0, a, b); trySeed(R - 1, a, b);
                trySeed(a, 0, b); trySeed(a, R - 1, b);
                trySeed(a, b, 0); trySeed(a, b, R - 1);
            }
        }

        const int dx[] = { 1,-1, 0, 0, 0, 0 };
        const int dy[] = { 0, 0, 1,-1, 0, 0 };
        const int dz[] = { 0, 0, 0, 0, 1,-1 };

        while (!q.empty()) {
            Node curr = q.front(); q.pop();
            for (int i = 0; i < 6; ++i) {
                int nx = curr.x + dx[i];
                int ny = curr.y + dy[i];
                int nz = curr.z + dz[i];
                if (grid.get(nx, ny, nz) == 0) {
                    grid.set(nx, ny, nz, 2);
                    q.push({ nx, ny, nz });
                }
            }
        }
    }

    // =============================================================================
    //  STAGE 2c — Extract watertight surface (FIX 2: check == 1, not != 2)
    // =============================================================================

    void wtExtractVoxelSurface(const WTVoxelGrid& grid, WTMesh& outMesh)
    {
        outMesh.vertices.clear();
        outMesh.faces.clear();

        std::map<std::tuple<int, int, int>, int> vertMap;
        auto getV = [&](int x, int y, int z) -> int {
            auto key = std::make_tuple(x, y, z);
            auto it = vertMap.find(key);
            if (it != vertMap.end()) return it->second;
            WTVec3 p = grid.gridToWorld(x, y, z);
            outMesh.vertices.push_back({ p.x, p.y, p.z });
            int idx = static_cast<int>(outMesh.vertices.size()) - 1;
            vertMap[key] = idx;
            return idx;
            };

        const int ddx[] = { 1,-1, 0, 0, 0, 0 };
        const int ddy[] = { 0, 0, 1,-1, 0, 0 };
        const int ddz[] = { 0, 0, 0, 0, 1,-1 };

        for (int x = 0; x < grid.res; ++x) {
            for (int y = 0; y < grid.res; ++y) {
                for (int z = 0; z < grid.res; ++z) {
                    // FIX 2: Only process voxels marked as SURFACE (value == 1)
                    if (grid.get(x, y, z) == 1) {
                        for (int i = 0; i < 6; ++i) {
                            if (grid.get(x + ddx[i], y + ddy[i], z + ddz[i]) != 2) continue;

                            WTFace f;
                            switch (i) {
                            case 0: f.vertexIndices = { getV(x + 1,y,  z), getV(x + 1,y + 1,z),
                                                        getV(x + 1,y + 1,z + 1), getV(x + 1,y,  z + 1) }; break;
                            case 1: f.vertexIndices = { getV(x,  y,  z), getV(x,  y,  z + 1),
                                                        getV(x,  y + 1,z + 1), getV(x,  y + 1,z) }; break;
                            case 2: f.vertexIndices = { getV(x,  y + 1,z), getV(x,  y + 1,z + 1),
                                                        getV(x + 1,y + 1,z + 1), getV(x + 1,y + 1,z) }; break;
                            case 3: f.vertexIndices = { getV(x,  y,  z), getV(x + 1,y,  z),
                                                        getV(x + 1,y,  z + 1), getV(x,  y,  z + 1) }; break;
                            case 4: f.vertexIndices = { getV(x,  y,  z + 1), getV(x + 1,y,  z + 1),
                                                        getV(x + 1,y + 1,z + 1), getV(x,  y + 1,z + 1) }; break;
                            default: f.vertexIndices = { getV(x,  y,  z), getV(x,  y + 1,z),
                                                         getV(x + 1,y + 1,z), getV(x + 1,y,  z) }; break;
                            }
                            outMesh.faces.push_back(f);
                        }
                    }
                }
            }
        }
    }

    // =============================================================================
    //  STAGE 2d — Laplacian smoothing
    // =============================================================================

    void wtSmoothMesh(WTMesh& mesh, int iterations)
    {
        for (int iter = 0; iter < iterations; ++iter) {
            std::vector<WTVec3> sums(mesh.vertices.size(), { 0.f, 0.f, 0.f });
            std::vector<int>    counts(mesh.vertices.size(), 0);

            for (const auto& f : mesh.faces) {
                const int n = static_cast<int>(f.vertexIndices.size());
                for (int i = 0; i < n; ++i) {
                    int a = f.vertexIndices[i];
                    int b = f.vertexIndices[(i + 1) % n];
                    sums[a].x += mesh.vertices[b].x;
                    sums[a].y += mesh.vertices[b].y;
                    sums[a].z += mesh.vertices[b].z;
                    sums[b].x += mesh.vertices[a].x;
                    sums[b].y += mesh.vertices[a].y;
                    sums[b].z += mesh.vertices[a].z;
                    counts[a]++;
                    counts[b]++;
                }
            }

            for (size_t i = 0; i < mesh.vertices.size(); ++i) {
                if (counts[i] > 0) {
                    float inv = 1.0f / counts[i];
                    mesh.vertices[i] = { sums[i].x * inv,
                                         sums[i].y * inv,
                                         sums[i].z * inv };
                }
            }
        }
    }

    // =============================================================================
    //  ALL-IN-ONE PIPELINE (FIX 3 & 5: respect voxelRes, save correct format)
    // =============================================================================

    std::string wtRepairMesh(
        const std::string& inputPath,
        const std::string& outputPath,
        int                voxelRes,
        WTProgressCallback progress)
    {
        auto report = [&](int pct, const std::string& stage) {
            if (progress) progress(pct, stage);
            };

        report(0, "Loading mesh");
        WTMesh mesh;
        if (!wtLoadMesh(inputPath, mesh))
            return "Failed to load mesh from: " + inputPath;
        if (mesh.vertices.empty())
            return "Mesh contains no vertices.";

        report(10, "Stage 1: Surface hole filling");
        wtFillHolesSurface(mesh);

        report(20, "Stage 2: Computing bounding box");
        WTBBox bbox = wtGetMeshBBox(mesh);

        // FIX 3: Use the passed voxelRes parameter (with minimum 128)
        const int internalRes = std::max(voxelRes, 128);

        report(25, "Stage 2: Initializing voxel grid");
        WTVoxelGrid grid(internalRes, bbox);

        report(30, "Stage 2: Rasterizing mesh into voxels");
        wtRasterizeMesh(mesh, grid);

        report(65, "Stage 2: Flood-filling exterior");
        wtFloodFillOutside(grid);

        report(80, "Stage 2: Extracting watertight surface");
        WTMesh repaired;
        wtExtractVoxelSurface(grid, repaired);

        if (repaired.vertices.empty())
            return "Voxel extraction produced an empty mesh. "
            "Try increasing the voxel resolution.";

        report(90, "Stage 2: Smoothing");
        wtSmoothMesh(repaired, 3);

        report(95, "Saving repaired mesh");
        // FIX 5: wtSaveMesh now handles STL/OBJ/PLY based on extension
        if (!wtSaveMesh(outputPath, repaired))
            return "Failed to save repaired mesh to: " + outputPath;

        report(100, "Done");
        return {};
    }

} // namespace Mayo