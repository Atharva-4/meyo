#include "StlHollowing.h"

#include <cmath>
#include <cstdint>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace StlHollowing {

    // ── Vec3 math helpers ─────────────────────────────────────────────────────

    Vec3 operator-(Vec3 a, Vec3 b) {
        return { a.x - b.x, a.y - b.y, a.z - b.z };
    }

    Vec3 operator+(Vec3 a, Vec3 b) {
        return { a.x + b.x, a.y + b.y, a.z + b.z };
    }

    Vec3 operator*(Vec3 v, float s) {
        return { v.x * s, v.y * s, v.z * s };
    }

    Vec3 cross(Vec3 a, Vec3 b) {
        return {
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x
        };
    }

    Vec3 normalize(Vec3 v) {
        const float len = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
        if (len < 1e-12f) return { 0.f, 0.f, 0.f };
        return { v.x / len, v.y / len, v.z / len };
    }

    Vec3 faceNormal(const Triangle& t) {
        return normalize(cross(t.v2 - t.v1, t.v3 - t.v1));
    }

    // ── File-format detection ─────────────────────────────────────────────────

    bool isFileBinary(const std::string& path) {
        std::ifstream f(path, std::ios::binary);
        if (!f) return false;

        f.seekg(0, std::ios::end);
        const std::streamsize sz = f.tellg();
        if (sz < 84) return false;

        f.seekg(80, std::ios::beg);
        uint32_t n = 0;
        f.read(reinterpret_cast<char*>(&n), 4);

        return sz == static_cast<std::streamsize>(84)
            + static_cast<std::streamsize>(n) * 50;
    }

    // ── STL loaders ───────────────────────────────────────────────────────────

    std::vector<Triangle> loadBinaryStl(std::ifstream& f, uint32_t numTri) {
        std::vector<Triangle> out;
        out.reserve(numTri);

        for (uint32_t i = 0; i < numTri; ++i) {
            float    buf[12];
            uint16_t attr;
            f.read(reinterpret_cast<char*>(buf), 48);
            f.read(reinterpret_cast<char*>(&attr), 2);

            // buf[0..2] = normal (ignored — we recompute from vertices)
            // buf[3..5] = v1,  buf[6..8] = v2,  buf[9..11] = v3
            out.push_back({
                { buf[3],  buf[4],  buf[5]  },
                { buf[6],  buf[7],  buf[8]  },
                { buf[9],  buf[10], buf[11] }
                });
        }
        return out;
    }

    std::vector<Triangle> loadAsciiStl(std::ifstream& f) {
        std::vector<Triangle> out;
        std::string word;

        while (f >> word) {
            if (word != "facet") continue;

            float n[3], v1[3], v2[3], v3[3];

            f >> word;                      // "normal"
            f >> n[0] >> n[1] >> n[2];

            f >> word >> word;              // "outer" "loop"

            f >> word;                      // "vertex"
            f >> v1[0] >> v1[1] >> v1[2];

            f >> word;                      // "vertex"
            f >> v2[0] >> v2[1] >> v2[2];

            f >> word;                      // "vertex"
            f >> v3[0] >> v3[1] >> v3[2];

            f >> word >> word;              // "endloop" "endfacet"

            out.push_back({
                { v1[0], v1[1], v1[2] },
                { v2[0], v2[1], v2[2] },
                { v3[0], v3[1], v3[2] }
                });
        }
        return out;
    }

    std::vector<Triangle> loadStlFile(const std::string& path) {
        std::ifstream f(path, std::ios::binary);
        if (!f)
            throw std::runtime_error("StlHollowing: cannot open '" + path + "'");

        f.seekg(0, std::ios::end);
        const std::streamsize sz = f.tellg();
        f.seekg(0, std::ios::beg);

        // Try binary first
        if (sz >= 84) {
            char     header[80];
            uint32_t n = 0;
            f.read(header, 80);
            f.read(reinterpret_cast<char*>(&n), 4);

            if (sz == static_cast<std::streamsize>(84)
                + static_cast<std::streamsize>(n) * 50)
                return loadBinaryStl(f, n);
        }

        // Fall back to ASCII
        f.seekg(0, std::ios::beg);
        return loadAsciiStl(f);
    }

    // ── STL writers ───────────────────────────────────────────────────────────

    void saveStlFileBinary(const std::string& path, const std::vector<Triangle>& tris) {
        std::ofstream f(path, std::ios::binary);
        if (!f)
            throw std::runtime_error("StlHollowing: cannot write '" + path + "'");

        // 80-byte header
        char header[80] = {};
        const char* title = "Hollowed STL - StlHollowing";
        std::memcpy(header, title, std::strlen(title));
        f.write(header, 80);

        // Triangle count
        const uint32_t count = static_cast<uint32_t>(tris.size());
        f.write(reinterpret_cast<const char*>(&count), 4);

        const uint16_t attr = 0;
        for (const auto& t : tris) {
            const Vec3 n = faceNormal(t);
            float data[12] = {
                n.x,    n.y,    n.z,
                t.v1.x, t.v1.y, t.v1.z,
                t.v2.x, t.v2.y, t.v2.z,
                t.v3.x, t.v3.y, t.v3.z
            };
            f.write(reinterpret_cast<const char*>(data), 48);
            f.write(reinterpret_cast<const char*>(&attr), 2);
        }
    }

    void saveStlFileAscii(const std::string& path, const std::vector<Triangle>& tris) {
        std::ofstream f(path);
        if (!f)
            throw std::runtime_error("StlHollowing: cannot write '" + path + "'");

        f << "solid hollow\n";
        for (const auto& t : tris) {
            const Vec3 n = faceNormal(t);
            f << "facet normal " << n.x << ' ' << n.y << ' ' << n.z << "\n"
                << "outer loop\n"
                << "vertex " << t.v1.x << ' ' << t.v1.y << ' ' << t.v1.z << "\n"
                << "vertex " << t.v2.x << ' ' << t.v2.y << ' ' << t.v2.z << "\n"
                << "vertex " << t.v3.x << ' ' << t.v3.y << ' ' << t.v3.z << "\n"
                << "endloop\n"
                << "endfacet\n";
        }
        f << "endsolid\n";
    }

    void saveStlFile(const std::string& path,
        const std::vector<Triangle>& tris,
        bool isBinary) {
        if (isBinary)
            saveStlFileBinary(path, tris);
        else
            saveStlFileAscii(path, tris);
    }

    // ── Core algorithms ───────────────────────────────────────────────────────

    Triangle offsetTriangle(const Triangle& t, float amount) {
        const Vec3 n = faceNormal(t) * amount;
        return {
            { t.v1.x + n.x, t.v1.y + n.y, t.v1.z + n.z },
            { t.v2.x + n.x, t.v2.y + n.y, t.v2.z + n.z },
            { t.v3.x + n.x, t.v3.y + n.y, t.v3.z + n.z }
        };
    }

    std::vector<Triangle> offsetTriangles(const std::vector<Triangle>& tris, float amount) {
        std::vector<Triangle> out;
        out.reserve(tris.size());
        for (const auto& t : tris)
            out.push_back(offsetTriangle(t, amount));
        return out;
    }

    std::vector<Triangle> buildHollowMesh(const std::vector<Triangle>& tris, float thickness) {
        return buildHollowMeshWithOffsets(tris, std::abs(thickness), 0.f);
    }


    std::vector<Triangle> buildHollowMeshWithOffsets(
        const std::vector<Triangle>& tris,
        float innerOffset,
        float outerOffset)
    {
        std::vector<Triangle> out;

        // ── STEP 1: Collect all unique vertices with welding ──
        const float epsilon = 1e-4f;
        const float invEps = 1.0f / epsilon;

        std::map<QuantizedVertex, int> vertexMap;  // quantized → index
        std::vector<Vec3> uniqueVerts;
        std::vector<std::vector<int>> vertToFaces; // which faces use this vertex

        // First pass: map original triangle vertices to unique indices
        std::vector<std::array<int, 3>> triVertexIndices; // per-triangle: [v0,v1,v2] indices
        triVertexIndices.reserve(tris.size());

        for (const auto& t : tris) {
            std::array<int, 3> indices;
            const Vec3* verts[3] = { &t.v1, &t.v2, &t.v3 };

            for (int i = 0; i < 3; ++i) {
                const Vec3& v = *verts[i];
                QuantizedVertex qv = {
                    static_cast<int>(std::round(v.x * invEps)),
                    static_cast<int>(std::round(v.y * invEps)),
                    static_cast<int>(std::round(v.z * invEps))
                };

                auto it = vertexMap.find(qv);
                if (it == vertexMap.end()) {
                    int newIdx = static_cast<int>(uniqueVerts.size());
                    vertexMap[qv] = newIdx;
                    uniqueVerts.push_back(v);
                    vertToFaces.emplace_back();
                    indices[i] = newIdx;
                }
                else {
                    indices[i] = it->second;
                }
                vertToFaces[indices[i]].push_back(static_cast<int>(triVertexIndices.size()));
            }
            triVertexIndices.push_back(indices);
        }

        // ── STEP 2: Compute VERTEX normals (averaged from adjacent faces) ──
        std::vector<Vec3> vertexNormals(uniqueVerts.size(), { 0,0,0 });

        for (size_t fi = 0; fi < tris.size(); ++fi) {
            const auto& t = tris[fi];
            Vec3 fn = faceNormal(t);  // Face normal
            const auto& indices = triVertexIndices[fi];

            for (int vi : indices) {
                vertexNormals[vi] = vertexNormals[vi] + fn;
            }
        }

        for (auto& vn : vertexNormals) {
            vn = normalize(vn);
        }

        // ── STEP 3: Create offset vertices ──
        const size_t originalVertCount = uniqueVerts.size();
        std::vector<Vec3> outerVerts = uniqueVerts;
        std::vector<Vec3> innerVerts;
        innerVerts.reserve(originalVertCount);

        // Outer shell: offset outward
        for (size_t i = 0; i < originalVertCount; ++i) {
            outerVerts[i] = uniqueVerts[i] + vertexNormals[i] * outerOffset;
        }

        // Inner shell: offset inward
        for (size_t i = 0; i < originalVertCount; ++i) {
            innerVerts.push_back(uniqueVerts[i] - vertexNormals[i] * innerOffset);
        }

        // ── STEP 4: Build triangles ──
        out.reserve(tris.size() * 2);

        for (size_t fi = 0; fi < tris.size(); ++fi) {
            const auto& indices = triVertexIndices[fi];

            // Outer face: original winding
            out.push_back({
                outerVerts[indices[0]],
                outerVerts[indices[1]],
                outerVerts[indices[2]]
                });

            // Inner face: REVERSED winding (v0↔v2 swap)
            out.push_back({
                innerVerts[indices[2]],  // swapped
                innerVerts[indices[1]],
                innerVerts[indices[0]]   // swapped
                });
        }

        return out;
    
    }
} // namespace StlHollowing
