// ============================================================
//  TriToQuad.cpp
//  Triangle-to-Quad conversion for Mayo mesh tools.
//
//  Algorithm overview
//  ------------------
//  1. Load mesh (STL binary/ASCII, OBJ, PLY).
//  2. Weld duplicate vertices so shared edges can be found.
//  3. Build EdgeFaceMap: canonical edge -> [face indices].
//  4. For every interior edge (shared by exactly 2 triangles):
//       a. Compute the dihedral angle between the two triangle normals.
//       b. If angle < threshold AND the resulting quad is convex, mark
//          both triangles as paired.
//  5. Emit quad faces for paired triangles, tri faces for unpaired ones.
//  6. Write OBJ output.
// ============================================================

#include "TriToQuad.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

namespace {

    // ─── Math helpers ────────────────────────────────────────────────────────────

    struct V3 {
        float x = 0, y = 0, z = 0;
        V3 operator+(const V3& o) const { return { x + o.x, y + o.y, z + o.z }; }
        V3 operator-(const V3& o) const { return { x - o.x, y - o.y, z - o.z }; }
        V3 operator*(float s)     const { return { x * s,   y * s,   z * s }; }
        bool operator<(const V3& o) const {
            if (x != o.x) return x < o.x;
            if (y != o.y) return y < o.y;
            return z < o.z;
        }
        bool operator==(const V3& o) const { return x == o.x && y == o.y && z == o.z; }
    };

    float dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
    V3   cross(V3 a, V3 b) {
        return { a.y * b.z - a.z * b.y,
                 a.z * b.x - a.x * b.z,
                 a.x * b.y - a.y * b.x };
    }
    float length(V3 v) { return std::sqrt(dot(v, v)); }
    V3   normalize(V3 v) {
        float l = length(v);
        return l > 1e-9f ? v * (1.f / l) : V3{};
    }

    // ─── Mesh ────────────────────────────────────────────────────────────────────

    struct Face {
        std::vector<int> vi; // vertex indices (3 = tri)
    };

    struct Mesh {
        std::vector<V3>   verts;
        std::vector<Face> faces;
    };

    // ─── Loaders ─────────────────────────────────────────────────────────────────

    bool loadOBJ(const std::string& path, Mesh& mesh) {
        std::ifstream f(path);
        if (!f) return false;
        std::string line;
        while (std::getline(f, line)) {
            std::istringstream ss(line);
            std::string tok; ss >> tok;
            if (tok == "v") {
                V3 v; ss >> v.x >> v.y >> v.z;
                mesh.verts.push_back(v);
            }
            else if (tok == "f") {
                Face face;
                std::string seg;
                while (ss >> seg) {
                    int idx = std::stoi(seg.substr(0, seg.find('/'))) - 1;
                    face.vi.push_back(idx);
                }
                if (face.vi.size() >= 3)
                    mesh.faces.push_back(face);
            }
        }
        return !mesh.verts.empty();
    }

    bool loadSTL(const std::string& path, Mesh& mesh) {
        std::ifstream f(path, std::ios::binary);
        if (!f) return false;

        // Detect ASCII vs binary
        char hdr[6] = {};
        f.read(hdr, 5);
        bool ascii = std::string(hdr, 5) == "solid";
        f.seekg(0);

        if (ascii) {
            std::string line;
            while (std::getline(f, line)) {
                std::istringstream ss(line);
                std::string w; ss >> w;
                if (w == "vertex") {
                    V3 v; ss >> v.x >> v.y >> v.z;
                    mesh.verts.push_back(v);
                    if (mesh.verts.size() % 3 == 0) {
                        int n = (int)mesh.verts.size();
                        mesh.faces.push_back({ {n - 3, n - 2, n - 1} });
                    }
                }
            }
        }
        else {
            f.seekg(80);
            uint32_t nf = 0;
            f.read(reinterpret_cast<char*>(&nf), 4);
            for (uint32_t i = 0; i < nf; ++i) {
                float dummy[3];
                f.read(reinterpret_cast<char*>(dummy), 12); // skip normal
                int base = (int)mesh.verts.size();
                for (int k = 0; k < 3; ++k) {
                    V3 v;
                    f.read(reinterpret_cast<char*>(&v.x), 4);
                    f.read(reinterpret_cast<char*>(&v.y), 4);
                    f.read(reinterpret_cast<char*>(&v.z), 4);
                    mesh.verts.push_back(v);
                }
                uint16_t attr; f.read(reinterpret_cast<char*>(&attr), 2);
                mesh.faces.push_back({ {base, base + 1, base + 2} });
            }
        }
        return !mesh.verts.empty();
    }

    bool loadPLY(const std::string& path, Mesh& mesh) {
        std::ifstream f(path);
        if (!f) return false;
        int nv = 0, nf = 0;
        std::string line;
        while (std::getline(f, line)) {
            if (line.find("element vertex") != std::string::npos)
                nv = std::stoi(line.substr(15));
            else if (line.find("element face") != std::string::npos)
                nf = std::stoi(line.substr(13));
            else if (line == "end_header") break;
        }
        for (int i = 0; i < nv; ++i) {
            V3 v; f >> v.x >> v.y >> v.z;
            mesh.verts.push_back(v);
        }
        for (int i = 0; i < nf; ++i) {
            int cnt; f >> cnt;
            Face face;
            for (int j = 0; j < cnt; ++j) { int idx; f >> idx; face.vi.push_back(idx); }
            if (face.vi.size() >= 3)
                mesh.faces.push_back(face);
        }
        return !mesh.verts.empty();
    }

    bool loadMesh(const std::string& path, Mesh& mesh) {
        auto ext = path.substr(path.rfind('.') + 1);
        for (auto& c : ext) c = (char)std::tolower((unsigned char)c);
        if (ext == "obj") return loadOBJ(path, mesh);
        if (ext == "stl") return loadSTL(path, mesh);
        if (ext == "ply") return loadPLY(path, mesh);
        return false;
    }

    // ─── Vertex welding ──────────────────────────────────────────────────────────

    // Re-index duplicate vertices (exact match) so edge sharing is detected correctly.
    void weldVertices(Mesh& mesh) {
        std::map<V3, int> uniq;
        std::vector<int> remap(mesh.verts.size());
        std::vector<V3>  welded;

        for (int i = 0; i < (int)mesh.verts.size(); ++i) {
            auto it = uniq.find(mesh.verts[i]);
            if (it == uniq.end()) {
                remap[i] = (int)welded.size();
                uniq[mesh.verts[i]] = remap[i];
                welded.push_back(mesh.verts[i]);
            }
            else {
                remap[i] = it->second;
            }
        }

        mesh.verts = std::move(welded);
        for (auto& face : mesh.faces)
            for (auto& vi : face.vi)
                vi = remap[vi];
    }

    // ─── Canonical edge ──────────────────────────────────────────────────────────

    using Edge2 = std::pair<int, int>;
    Edge2 makeEdge(int a, int b) {
        return a < b ? Edge2{ a,b } : Edge2{ b,a };
    }

    // ─── Triangle normal ─────────────────────────────────────────────────────────

    V3 triNormal(const Mesh& m, const Face& f) {
        V3 a = m.verts[f.vi[0]];
        V3 b = m.verts[f.vi[1]];
        V3 c = m.verts[f.vi[2]];
        return normalize(cross(b - a, c - a));
    }

    // ─── Convexity check ─────────────────────────────────────────────────────────
    // Given two triangles sharing edge (ev0, ev1), check if the merged quad is
    // strictly convex (all cross products same sign when traversed in order).
    bool isConvexQuad(const std::vector<V3>& verts,
        const std::array<int, 4>& quad) {
        // Compute winding in a consistent plane
        V3 n = {};
        for (int i = 0; i < 4; ++i) {
            V3 e0 = verts[quad[(i + 1) % 4]] - verts[quad[i]];
            V3 e1 = verts[quad[(i + 2) % 4]] - verts[quad[(i + 1) % 4]];
            n = n + cross(e0, e1);
        }
        float nl = length(n);
        if (nl < 1e-9f) return false;
        n = n * (1.f / nl);

        for (int i = 0; i < 4; ++i) {
            V3 e0 = verts[quad[(i + 1) % 4]] - verts[quad[i]];
            V3 e1 = verts[quad[(i + 2) % 4]] - verts[quad[(i + 1) % 4]];
            if (dot(cross(e0, e1), n) < 0.f) return false;
        }
        return true;
    }

    // ─── Build merged quad from two triangles sharing edge (ev0,ev1) ─────────────
    // Returns false if a valid convex quad cannot be built.
    bool buildQuad(const Mesh& mesh,
        int fi, int fj,
        int ev0, int ev1,
        std::array<int, 4>& outQuad) {
        // Find the vertex of fi NOT on the shared edge
        int apexI = -1;
        for (int vi : mesh.faces[fi].vi)
            if (vi != ev0 && vi != ev1) { apexI = vi; break; }

        int apexJ = -1;
        for (int vi : mesh.faces[fj].vi)
            if (vi != ev0 && vi != ev1) { apexJ = vi; break; }

        if (apexI == -1 || apexJ == -1) return false;

        // Wind the quad: apexI -> ev0 -> apexJ -> ev1  (or the other orientation)
        // We need to determine the correct winding from face fi's order.
        // Walk fi's vertices to find ev0->ev1 or ev1->ev0 order.
        int n = (int)mesh.faces[fi].vi.size();
        bool ev0BeforeEv1 = false;
        for (int k = 0; k < n; ++k) {
            if (mesh.faces[fi].vi[k] == ev0 && mesh.faces[fi].vi[(k + 1) % n] == ev1) {
                ev0BeforeEv1 = true; break;
            }
        }

        // Build candidate quad winding
        if (ev0BeforeEv1) {
            outQuad = { apexI, ev0, apexJ, ev1 };
        }
        else {
            outQuad = { apexI, ev1, apexJ, ev0 };
        }

        return isConvexQuad(mesh.verts, outQuad);
    }

    // ─── OBJ writer ──────────────────────────────────────────────────────────────

    bool saveOBJ(const std::string& path, const Mesh& mesh) {
        std::ofstream f(path);
        if (!f) return false;
        f << "# Tri-to-Quad converted mesh by Mayo TriToQuad\n";
        for (const auto& v : mesh.verts)
            f << "v " << v.x << " " << v.y << " " << v.z << "\n";
        for (const auto& face : mesh.faces) {
            f << "f";
            for (int vi : face.vi)
                f << " " << (vi + 1);
            f << "\n";
        }
        return true;
    }

} // anonymous namespace

// ─── Public API ──────────────────────────────────────────────────────────────

namespace Mayo {

    std::string triToQuadConvert(
        const std::string& inputPath,
        const std::string& outputPath,
        float angleThresholdDeg,
        T2QProgressCallback cb)
    {
        auto report = [&](int pct, const std::string& msg) {
            if (cb) cb(pct, msg);
            };

        // 1. Load
        report(0, "Loading mesh…");
        Mesh mesh;
        if (!loadMesh(inputPath, mesh))
            return "Failed to load mesh from: " + inputPath;

        // Only process triangle faces; pass quads/ngons through unchanged
        // (e.g. if the user re-runs on an already-converted mesh)
        report(5, "Welding vertices…");
        weldVertices(mesh);

        const int nFaces = (int)mesh.faces.size();
        if (nFaces == 0)
            return "Mesh contains no faces.";

        report(10, "Building edge adjacency…");

        // edge -> list of face indices that use it
        std::map<Edge2, std::vector<int>> edgeFaceMap;
        for (int fi = 0; fi < nFaces; ++fi) {
            const auto& f = mesh.faces[fi];
            if (f.vi.size() != 3) continue; // skip non-tris
            int n = (int)f.vi.size();
            for (int k = 0; k < n; ++k) {
                Edge2 e = makeEdge(f.vi[k], f.vi[(k + 1) % n]);
                edgeFaceMap[e].push_back(fi);
            }
        }

        report(20, "Pairing triangles into quads…");

        const float cosThresh = std::cos(angleThresholdDeg * 3.14159265f / 180.f);
        std::vector<bool> paired(nFaces, false);
        std::vector<Face> quadFaces; // newly merged quad faces

        int edgesDone = 0;
        int totalInterior = 0;
        for (auto& [e, flist] : edgeFaceMap)
            if (flist.size() == 2) ++totalInterior;

        for (auto& [e, flist] : edgeFaceMap) {
            if (flist.size() != 2) continue;

            int fi = flist[0], fj = flist[1];
            if (paired[fi] || paired[fj]) continue;
            if (mesh.faces[fi].vi.size() != 3) continue;
            if (mesh.faces[fj].vi.size() != 3) continue;

            // Planarity check
            V3 ni = triNormal(mesh, mesh.faces[fi]);
            V3 nj = triNormal(mesh, mesh.faces[fj]);
            if (dot(ni, nj) < cosThresh) continue;

            // Convexity check + quad winding
            std::array<int, 4> quad{};
            if (!buildQuad(mesh, fi, fj, e.first, e.second, quad))
                continue;

            // Accept pair
            paired[fi] = paired[fj] = true;
            Face qf;
            qf.vi = { quad[0], quad[1], quad[2], quad[3] };
            quadFaces.push_back(std::move(qf));

            ++edgesDone;
            if (totalInterior > 0 && edgesDone % std::max(1, totalInterior / 50) == 0) {
                int pct = 20 + (int)(60.f * edgesDone / totalInterior);
                report(pct, "Merging pairs…");
            }
        }

        // 5. Build output mesh: quads first, then unpaired tris
        report(82, "Building output mesh…");
        Mesh outMesh;
        outMesh.verts = mesh.verts;

        for (auto& qf : quadFaces)
            outMesh.faces.push_back(std::move(qf));

        int triKept = 0;
        for (int fi = 0; fi < nFaces; ++fi) {
            if (!paired[fi]) {
                outMesh.faces.push_back(mesh.faces[fi]);
                ++triKept;
            }
        }

        int quadsCreated = (int)quadFaces.size();
        int trisConverted = quadsCreated * 2;

        report(90, "Saving OBJ…");
        if (!saveOBJ(outputPath, outMesh))
            return "Failed to write output file: " + outputPath;

        report(100,
            "Done. Quads: " + std::to_string(quadsCreated) +
            "  Remaining tris: " + std::to_string(triKept) +
            "  (" + std::to_string(trisConverted) + " triangles merged)");

        return ""; // success
    }

} // namespace Mayo