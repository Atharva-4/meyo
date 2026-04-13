#pragma once

#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

namespace StlHollowing {

    // ── Basic geometry types ──────────────────────────────────────────────────

    struct Vec3 {
        float x = 0, y = 0, z = 0;
    };

    struct Triangle {
        Vec3 v1, v2, v3;
    };

    // ── Vec3 math helpers ─────────────────────────────────────────────────────

    Vec3 operator-(Vec3 a, Vec3 b);
    Vec3 operator+(Vec3 a, Vec3 b);
    Vec3 operator*(Vec3 v, float s);

    Vec3 cross(Vec3 a, Vec3 b);
    Vec3 normalize(Vec3 v);
    Vec3 faceNormal(const Triangle& t);

    // ── File-format detection ─────────────────────────────────────────────────

    /// Returns true if the file passes the binary-STL size check.
    bool isFileBinary(const std::string& path);

    // ── STL loaders ───────────────────────────────────────────────────────────

    /// Internal loaders (called by loadStlFile)
    std::vector<Triangle> loadBinaryStl(std::ifstream& f, uint32_t numTri);
    std::vector<Triangle> loadAsciiStl(std::ifstream& f);

    /// Load triangles from any STL file (auto-detects binary or ASCII).
    std::vector<Triangle> loadStlFile(const std::string& path);

    // ── STL writers ───────────────────────────────────────────────────────────

    void saveStlFileBinary(const std::string& path, const std::vector<Triangle>& tris);
    void saveStlFileAscii(const std::string& path, const std::vector<Triangle>& tris);

    /// Save triangles to STL.
    /// isBinary = true  → binary STL
    /// isBinary = false → ASCII  STL
    void saveStlFile(const std::string& path,
        const std::vector<Triangle>& tris,
        bool isBinary = true);

    // ── Core algorithms ───────────────────────────────────────────────────────

    /// Offset a single triangle along its face normal by `amount`.
    /// Positive amount → outward expansion.
    /// Negative amount → inward shrink.
    Triangle offsetTriangle(const Triangle& t, float amount);

    /// Offset every triangle by `amount` along its own face normal.
    std::vector<Triangle> offsetTriangles(const std::vector<Triangle>& tris, float amount);

    /// Build a hollow mesh:
    ///   - Outer shell  : original triangles, unchanged winding.
    ///   - Inner shell  : inward-offset triangles, winding reversed
    ///                    so normals point into the hollow cavity.
    /// `thickness` must be positive.
    std::vector<Triangle> buildHollowMesh(const std::vector<Triangle>& tris, float thickness);

} // namespace StlHollowing
