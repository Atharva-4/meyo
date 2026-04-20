// ============================================================
//  QuadRemesher.cpp  — v3: robust greedy quad pairing
//
//  Strategy change from v2:
//  UV-grid matching is fragile at high-curvature regions and
//  produces distorted / flipped faces there.
//
//  New extraction strategy:
//  1. Compute RoSy field (same as before — gives preferred directions)
//  2. For every interior edge, score the triangle pair by:
//       a. How coplanar they are          (dihedral angle cost)
//       b. How well-aligned the shared edge is with the RoSy field
//          (edges perpendicular to the field direction make good quads)
//       c. How convex the resulting quad is
//  3. Sort all candidate pairs by score (best first)
//  4. Greedily accept pairs — each triangle used at most once
//  5. Remaining unpaired triangles → kept as tris
//
//  This removes all UV integer rounding issues and works correctly
//  at high-curvature regions (ears, sharp edges).
//  Winding is always derived from the original face winding → no flips.
// ============================================================

#include "QuadRemesher.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <functional>
#include <map>
#include <numeric>
#include <queue>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace {

    // ─── Math ─────────────────────────────────────────────────────────────────────

    struct V3 {
        float x = 0, y = 0, z = 0;
        V3() = default;
        V3(float x, float y, float z) :x(x), y(y), z(z) {}
        V3 operator+(const V3& o)const { return{ x + o.x,y + o.y,z + o.z }; }
        V3 operator-(const V3& o)const { return{ x - o.x,y - o.y,z - o.z }; }
        V3 operator*(float s)    const { return{ x * s,y * s,z * s }; }
        V3& operator+=(const V3& o) { x += o.x; y += o.y; z += o.z; return*this; }
        bool operator<(const V3& o)const {
            if (x != o.x)return x < o.x;
            if (y != o.y)return y < o.y;
            return z < o.z;
        }
    };

    float dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
    V3 cross(V3 a, V3 b) { return{ a.y * b.z - a.z * b.y,a.z * b.x - a.x * b.z,a.x * b.y - a.y * b.x }; }
    float len(V3 v) { return std::sqrt(dot(v, v)); }
    V3 normalize(V3 v) { float l = len(v); return l > 1e-9f ? v * (1.f / l) : V3{}; }
    V3 projectTangent(V3 v, V3 n) { return v - n * dot(v, n); }

    // ─── Mesh ─────────────────────────────────────────────────────────────────────

    struct Face { int v[3]; };
    struct Mesh { std::vector<V3> verts; std::vector<Face> faces; };

    // ─── Loaders ──────────────────────────────────────────────────────────────────

    bool loadOBJ(const std::string& path, Mesh& mesh) {
        std::ifstream f(path); if (!f)return false;
        std::string line;
        while (std::getline(f, line)) {
            std::istringstream ss(line); std::string tok; ss >> tok;
            if (tok == "v") { V3 v; ss >> v.x >> v.y >> v.z; mesh.verts.push_back(v); }
            else if (tok == "f") {
                std::vector<int> idx; std::string seg;
                while (ss >> seg)idx.push_back(std::stoi(seg.substr(0, seg.find('/'))) - 1);
                for (int i = 1; i + 1 < (int)idx.size(); ++i)
                    mesh.faces.push_back({ idx[0],idx[i],idx[i + 1] });
            }
        }
        return !mesh.verts.empty();
    }

    bool loadSTL(const std::string& path, Mesh& mesh) {
        std::ifstream f(path, std::ios::binary); if (!f)return false;
        char hdr[6] = {}; f.read(hdr, 5);
        bool ascii = std::string(hdr, 5) == "solid"; f.seekg(0);
        if (ascii) {
            std::string line;
            while (std::getline(f, line)) {
                std::istringstream ss(line); std::string w; ss >> w;
                if (w == "vertex") {
                    V3 v; ss >> v.x >> v.y >> v.z; mesh.verts.push_back(v);
                    int n = (int)mesh.verts.size();
                    if (n % 3 == 0)mesh.faces.push_back({ n - 3,n - 2,n - 1 });
                }
            }
        }
        else {
            f.seekg(80); uint32_t nf = 0; f.read((char*)&nf, 4);
            for (uint32_t i = 0; i < nf; ++i) {
                float dummy[3]; f.read((char*)dummy, 12);
                int base = (int)mesh.verts.size();
                for (int k = 0; k < 3; ++k) {
                    V3 v; f.read((char*)&v.x, 4); f.read((char*)&v.y, 4); f.read((char*)&v.z, 4);
                    mesh.verts.push_back(v);
                }
                uint16_t attr; f.read((char*)&attr, 2);
                mesh.faces.push_back({ base,base + 1,base + 2 });
            }
        }
        return !mesh.verts.empty();
    }

    bool loadPLY(const std::string& path, Mesh& mesh) {
        std::ifstream f(path); if (!f)return false;
        int nv = 0, nf = 0; std::string line;
        while (std::getline(f, line)) {
            if (line.find("element vertex") != std::string::npos)nv = std::stoi(line.substr(15));
            else if (line.find("element face") != std::string::npos)nf = std::stoi(line.substr(13));
            else if (line == "end_header")break;
        }
        for (int i = 0; i < nv; ++i) { V3 v; f >> v.x >> v.y >> v.z; mesh.verts.push_back(v); }
        for (int i = 0; i < nf; ++i) {
            int cnt; f >> cnt; int a, b, c; f >> a >> b >> c;
            mesh.faces.push_back({ a,b,c });
            for (int j = 3; j < cnt; ++j) { int x; f >> x; }
        }
        return !mesh.verts.empty();
    }

    bool loadMesh(const std::string& path, Mesh& mesh) {
        auto ext = path.substr(path.rfind('.') + 1);
        for (auto& c : ext)c = (char)std::tolower((unsigned char)c);
        if (ext == "obj")return loadOBJ(path, mesh);
        if (ext == "stl")return loadSTL(path, mesh);
        if (ext == "ply")return loadPLY(path, mesh);
        return false;
    }

    // ─── Vertex welding ───────────────────────────────────────────────────────────

    void weldVertices(Mesh& mesh) {
        std::map<V3, int> uniq;
        std::vector<int> remap(mesh.verts.size());
        std::vector<V3> welded;
        for (int i = 0; i < (int)mesh.verts.size(); ++i) {
            auto it = uniq.find(mesh.verts[i]);
            if (it == uniq.end()) {
                remap[i] = (int)welded.size();
                uniq[mesh.verts[i]] = remap[i];
                welded.push_back(mesh.verts[i]);
            }
            else remap[i] = it->second;
        }
        mesh.verts = std::move(welded);
        for (auto& f : mesh.faces)for (int k = 0; k < 3; ++k)f.v[k] = remap[f.v[k]];
    }

    // ─── Per-face normal ──────────────────────────────────────────────────────────

    V3 faceNormal(const Mesh& m, int fi) {
        V3 a = m.verts[m.faces[fi].v[0]];
        V3 b = m.verts[m.faces[fi].v[1]];
        V3 c = m.verts[m.faces[fi].v[2]];
        return normalize(cross(b - a, c - a));
    }

    // ─── Vertex normals ───────────────────────────────────────────────────────────

    std::vector<V3> computeVertexNormals(const Mesh& mesh) {
        int nv = (int)mesh.verts.size();
        std::vector<V3> vn(nv);
        for (const auto& f : mesh.faces) {
            V3 a = mesh.verts[f.v[0]], b = mesh.verts[f.v[1]], c = mesh.verts[f.v[2]];
            V3 fn = cross(b - a, c - a);
            for (int k = 0; k < 3; ++k)vn[f.v[k]] += fn;
        }
        for (auto& n : vn)n = normalize(n);
        return vn;
    }

    // ─── Vertex adjacency ─────────────────────────────────────────────────────────

    std::vector<std::vector<int>> buildVertexAdj(const Mesh& mesh) {
        int nv = (int)mesh.verts.size();
        std::vector<std::set<int>> adjSet(nv);
        for (const auto& f : mesh.faces) {
            adjSet[f.v[0]].insert(f.v[1]); adjSet[f.v[1]].insert(f.v[0]);
            adjSet[f.v[1]].insert(f.v[2]); adjSet[f.v[2]].insert(f.v[1]);
            adjSet[f.v[0]].insert(f.v[2]); adjSet[f.v[2]].insert(f.v[0]);
        }
        std::vector<std::vector<int>> adj(nv);
        for (int i = 0; i < nv; ++i)adj[i].assign(adjSet[i].begin(), adjSet[i].end());
        return adj;
    }

    // ─── Edge → face map ─────────────────────────────────────────────────────────

    using EKey = std::pair<int, int>;
    EKey ekey(int a, int b) { return a < b ? EKey{ a,b } : EKey{ b,a }; }

    std::map<EKey, std::vector<int>> buildEdgeFaceMap(const Mesh& mesh) {
        std::map<EKey, std::vector<int>> m;
        for (int fi = 0; fi < (int)mesh.faces.size(); ++fi) {
            const auto& f = mesh.faces[fi];
            m[ekey(f.v[0], f.v[1])].push_back(fi);
            m[ekey(f.v[1], f.v[2])].push_back(fi);
            m[ekey(f.v[0], f.v[2])].push_back(fi);
        }
        return m;
    }

    // ─── 4-RoSy field ─────────────────────────────────────────────────────────────

    V3 rosy4Rotate(V3 d, V3 n, int k) {
        for (int i = 0; i < (k & 3); ++i)d = cross(n, d);
        return d;
    }

    V3 bestMatch4(V3 ref, V3 fj, V3 ni) {
        V3 tj = normalize(projectTangent(fj, ni));
        float best = -2.f; V3 bestD = tj;
        for (int k = 0; k < 4; ++k) {
            V3 c = rosy4Rotate(tj, ni, k);
            float d = dot(ref, c);
            if (d > best) { best = d; bestD = c; }
        }
        return bestD;
    }

    std::vector<V3> computeRoSyField(
        const Mesh& mesh,
        const std::vector<V3>& vn,
        const std::vector<std::vector<int>>& adj,
        int iterations,
        const std::function<void(int, int)>& progress)
    {
        int nv = (int)mesh.verts.size();
        std::vector<V3> field(nv);
        for (int i = 0; i < nv; ++i) {
            V3 n = vn[i];
            V3 seed = (std::abs(n.x) < 0.9f) ? V3{ 1,0,0 } : V3{ 0,1,0 };
            field[i] = normalize(projectTangent(seed, n));
        }
        for (int iter = 0; iter < iterations; ++iter) {
            for (int i = 0; i < nv; ++i) {
                if (adj[i].empty())continue;
                V3 n = vn[i];
                V3 sum{};
                for (int j : adj[i])sum += bestMatch4(field[i], field[j], n);
                V3 nd = normalize(projectTangent(sum, n));
                if (len(nd) > 1e-6f)field[i] = nd;
            }
            if (iter % 5 == 0)progress(iter, iterations);
        }
        return field;
    }

    // ─── Convexity check ─────────────────────────────────────────────────────────
    // Returns true if the quad (v0,v1,v2,v3) is convex.
    bool isConvex(const V3& p0, const V3& p1, const V3& p2, const V3& p3) {
        // Compute quad normal from first triangle
        V3 qn = normalize(cross(p1 - p0, p2 - p0));
        // All cross products of consecutive edges must point same side as qn
        V3 pts[4] = { p0,p1,p2,p3 };
        for (int i = 0; i < 4; ++i) {
            V3 e0 = pts[(i + 1) % 4] - pts[i];
            V3 e1 = pts[(i + 2) % 4] - pts[(i + 1) % 4];
            if (dot(cross(e0, e1), qn) < -1e-5f)return false;
        }
        return true;
    }

    // ─── Quad planarity score ─────────────────────────────────────────────────────
    // 1.0 = perfectly planar, 0.0 = very non-planar
    float planarityScore(const V3& p0, const V3& p1, const V3& p2, const V3& p3) {
        V3 n1 = normalize(cross(p1 - p0, p2 - p0));
        V3 n2 = normalize(cross(p2 - p0, p3 - p0));
        float d = dot(n1, n2);
        return (d + 1.f) * 0.5f; // remap [-1,1] → [0,1]
    }

    // ─── RoSy alignment score for an edge ────────────────────────────────────────
    // Good quad edges are PERPENDICULAR to the field direction.
    // i.e. dot(edgeDir, fieldDir) ≈ 0  →  score near 1.0
    float rosyAlignScore(V3 edgeDir, V3 fieldDir) {
        float d = std::abs(dot(normalize(edgeDir), normalize(fieldDir)));
        // d=0 → perpendicular (good),  d=1 → parallel (bad)
        return 1.f - d;
    }

    // ─── Build correct quad winding from two triangles ───────────────────────────
    // Returns {v0,v1,v2,v3} in CCW order, or empty on failure.
    std::vector<int> buildQuadWinding(
        const Mesh& mesh,
        int fi, int fj,
        int ev0, int ev1)
    {
        const auto& A = mesh.faces[fi];

        // Find apices
        int apexA = -1, apexB = -1;
        for (int k = 0; k < 3; ++k) {
            if (A.v[k] != ev0 && A.v[k] != ev1)apexA = A.v[k];
        }
        const auto& B = mesh.faces[fj];
        for (int k = 0; k < 3; ++k) {
            if (B.v[k] != ev0 && B.v[k] != ev1)apexB = B.v[k];
        }
        if (apexA < 0 || apexB < 0)return{};

        // Determine winding from face A:
        // find whether ev0→ev1 appears in A's winding
        bool ev0first = false;
        for (int k = 0; k < 3; ++k) {
            if (A.v[k] == ev0 && A.v[(k + 1) % 3] == ev1) { ev0first = true; break; }
        }

        // Build quad: apexA, ev0, apexB, ev1  OR  apexA, ev1, apexB, ev0
        std::vector<int> quad;
        if (ev0first) {
            // A winds: ...→apexA→ev0→ev1→...  so quad: apexA,ev0,apexB,ev1
            quad = { apexA,ev0,apexB,ev1 };
        }
        else {
            quad = { apexA,ev1,apexB,ev0 };
        }

        // Verify convexity; if not convex try the other winding
        const auto& vt = mesh.verts;
        if (!isConvex(vt[quad[0]], vt[quad[1]], vt[quad[2]], vt[quad[3]])) {
            // Try swapping apexA and apexB
            std::swap(quad[0], quad[2]);
            if (!isConvex(vt[quad[0]], vt[quad[1]], vt[quad[2]], vt[quad[3]]))
                return{}; // not convex either way — reject
        }

        return quad;
    }

    // ─── Main: greedy quad extraction ────────────────────────────────────────────

    struct OutFace { std::vector<int> vi; };

    std::vector<OutFace> extractQuadsGreedy(
        const Mesh& mesh,
        const std::vector<V3>& field,
        const std::vector<V3>& vn,
        const std::map<EKey, std::vector<int>>& edgeFaceMap,
        float dihedralThreshDeg)  // max angle between face normals to accept pairing
    {
        const float cosThresh = std::cos(dihedralThreshDeg * (float)M_PI / 180.f);

        // Collect and score all candidate pairs
        struct Candidate {
            float score;       // higher = better quad
            int   fi, fj;
            int   ev0, ev1;
        };
        std::vector<Candidate> candidates;
        candidates.reserve(edgeFaceMap.size());

        for (const auto& [e, fl] : edgeFaceMap) {
            if (fl.size() != 2)continue;
            int fi = fl[0], fj = fl[1];

            // Planarity: normals must be close
            V3 ni = faceNormal(mesh, fi);
            V3 nj = faceNormal(mesh, fj);
            float nDot = dot(ni, nj);
            if (nDot < cosThresh)continue; // too non-planar

            // Try to build the quad winding
            auto quad = buildQuadWinding(mesh, fi, fj, e.first, e.second);
            if (quad.empty())continue; // non-convex

            // Planarity score (using quad diagonals — more accurate)
            const auto& vt = mesh.verts;
            float plan = planarityScore(vt[quad[0]], vt[quad[1]], vt[quad[2]], vt[quad[3]]);

            // RoSy alignment: score how well the shared edge aligns with the field
            V3 edgeVec = vt[e.second] - vt[e.first];
            // Average field at the two edge vertices
            V3 fi0 = field[e.first], fi1 = field[e.second];
            V3 avgField = normalize(fi0 + bestMatch4(fi0, fi1, vn[e.first]));
            float align = rosyAlignScore(edgeVec, avgField);

            // Combined score: planarity is most important, then alignment
            float score = plan * 0.6f + align * 0.4f;

            candidates.push_back({ score,fi,fj,e.first,e.second });
        }

        // Sort best first
        std::sort(candidates.begin(), candidates.end(),
            [](const Candidate& a, const Candidate& b) {return a.score > b.score; });

        // Greedy assignment
        int nf = (int)mesh.faces.size();
        std::vector<bool> used(nf, false);
        std::vector<OutFace> result;
        result.reserve(nf / 2 + nf % 2);

        for (const auto& c : candidates) {
            if (used[c.fi] || used[c.fj])continue;
            auto quad = buildQuadWinding(mesh, c.fi, c.fj, c.ev0, c.ev1);
            if (quad.empty())continue;
            used[c.fi] = used[c.fj] = true;
            result.push_back({ quad });
        }

        // Remaining unpaired triangles
        for (int fi = 0; fi < nf; ++fi) {
            if (!used[fi]) {
                const auto& f = mesh.faces[fi];
                result.push_back({ {f.v[0],f.v[1],f.v[2]} });
            }
        }

        return result;
    }

    // ─── OBJ writer ───────────────────────────────────────────────────────────────

    bool saveOBJ(const std::string& path,
        const std::vector<V3>& verts,
        const std::vector<OutFace>& faces)
    {
        std::ofstream f(path); if (!f)return false;
        f << "# QuadRemesher — greedy RoSy-guided quad extraction\n";
        for (const auto& v : verts)f << "v " << v.x << " " << v.y << " " << v.z << "\n";
        int quads = 0, tris = 0;
        for (const auto& face : faces) {
            f << "f";
            for (int vi : face.vi)f << " " << (vi + 1);
            f << "\n";
            if ((int)face.vi.size() == 4)++quads; else++tris;
        }
        f << "# Quads:" << quads << " Tris:" << tris << "\n";
        return true;
    }

} // anonymous namespace

// ─── Public API ───────────────────────────────────────────────────────────────
//
//  Parameter meanings (v4):
//
//  targetFaceCount  — now repurposed as "quad quality" slider.
//                     LOW  value (e.g. 500)  = strict planarity = fewer but
//                     cleaner quads, more remaining triangles.
//                     HIGH value (e.g. 8000) = loose planarity = maximum quads,
//                     even slightly non-planar pairs accepted.
//                     The mapping is LINEAR: 100→5°, 10000→60°.
//                     At 60° almost every adjacent pair becomes a quad.
//
//  smoothIter       — RoSy field smoothing passes. More = better field
//                     alignment = more quads aligned to surface flow.
//                     50 is good, 100+ for complex organic shapes.

namespace Mayo {

    std::string quadRemesh(
        const std::string& inputPath,
        const std::string& outputPath,
        int  targetFaceCount,   // 100..10000 — higher = more (and smaller) quads
        int  smoothIter,        // RoSy smoothing iterations
        QRProgressCallback cb)
    {
        auto report = [&](int pct, const std::string& msg) {if (cb)cb(pct, msg); };

        // Map targetFaceCount → dihedral angle threshold
        // HIGH count → BIG angle → ACCEPT MORE pairs → MORE quads (what user wants)
        // LOW  count → SMALL angle → stricter → FEWER quads but cleaner
        // Range: 100 → 5°,  10000 → 60°
        float t = (float)(std::min(std::max(targetFaceCount, 100), 10000) - 100)
            / (10000.f - 100.f);          // 0..1
        float dihedralDeg = 5.f + t * 55.f;    // 5°..60°

        report(0, "Loading mesh…");
        Mesh mesh;
        if (!loadMesh(inputPath, mesh))return "Failed to load: " + inputPath;
        if (mesh.faces.empty())return "Mesh has no faces.";

        report(5, "Welding vertices…");
        weldVertices(mesh);
        if (mesh.faces.empty())return "No faces after weld.";

        report(8, "Building adjacency…");
        auto vertAdj = buildVertexAdj(mesh);
        auto edgeFaceMap = buildEdgeFaceMap(mesh);

        report(12, "Computing normals…");
        auto vn = computeVertexNormals(mesh);

        report(15, "Computing 4-RoSy orientation field…");
        auto rosyProg = [&](int iter, int total) {
            int pct = 15 + (int)(55.f * iter / std::max(1, total));
            report(pct, "Smoothing RoSy field (" +
                std::to_string(iter) + "/" + std::to_string(total) + ")…");
            };
        auto field = computeRoSyField(mesh, vn, vertAdj, smoothIter, rosyProg);

        report(72,
            "Pairing triangles into quads (dihedral ≤ " +
            std::to_string((int)dihedralDeg) + "°)…");
        auto outFaces = extractQuadsGreedy(mesh, field, vn, edgeFaceMap, dihedralDeg);

        int quads = 0, tris = 0;
        for (const auto& f : outFaces)(f.vi.size() == 4 ? quads : tris)++;

        report(93, "Saving OBJ…");
        if (!saveOBJ(outputPath, mesh.verts, outFaces))
            return "Failed to write: " + outputPath;

        int pct100 = (quads + tris > 0) ? (100 * quads / (quads + tris)) : 0;
        report(100,
            "Done. Quads: " + std::to_string(quads) +
            "  Tris: " + std::to_string(tris) +
            "  (" + std::to_string(pct100) + "% quad)");
        return "";
    }

} // namespace Mayo