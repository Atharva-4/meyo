#pragma once

#include <string>
#include <vector>
#include <map>
#include <set>
#include <tuple>
#include <functional>
#include <cstdint>

namespace Mayo {

    // ─────────────────────────────────────────────────────────────────────────────
    //  Basic geometry types (self-contained, no Windows/OpenCASCADE dependencies)
    // ─────────────────────────────────────────────────────────────────────────────

    struct WTVertex { float x, y, z; };
    struct WTFace { std::vector<int> vertexIndices; };
    struct WTMesh { std::vector<WTVertex> vertices; std::vector<WTFace> faces; };

    struct WTVec3 {
        float x, y, z;
        WTVec3 operator-(const WTVec3& v) const { return { x - v.x, y - v.y, z - v.z }; }
        WTVec3 operator+(const WTVec3& v) const { return { x + v.x, y + v.y, z + v.z }; }
        WTVec3 operator*(float s)          const { return { x * s,   y * s,   z * s }; }
        bool   operator<(const WTVec3& o)  const {
            if (x != o.x) return x < o.x;
            if (y != o.y) return y < o.y;
            return z < o.z;
        }
    };

    struct WTEdge {
        int v[2];
        WTEdge(int a, int b) { v[0] = std::min(a, b); v[1] = std::max(a, b); }
        bool operator<(const WTEdge& o) const {
            return v[0] != o.v[0] ? v[0] < o.v[0] : v[1] < o.v[1];
        }
    };

    struct WTBBox {
        WTVec3 min, max;
        bool intersects(const WTBBox& o) const {
            return (min.x <= o.max.x && max.x >= o.min.x) &&
                (min.y <= o.max.y && max.y >= o.min.y) &&
                (min.z <= o.max.z && max.z >= o.min.z);
        }
    };

    // ─────────────────────────────────────────────────────────────────────────────
    //  Voxel grid used during volumetric repair
    // ─────────────────────────────────────────────────────────────────────────────

    struct WTVoxelGrid {
        int res;
        std::vector<uint8_t> data;   // 0=empty, 1=surface, 2=outside
        WTVec3 minBound, maxBound, scale;

        WTVoxelGrid(int r, const WTBBox& box);
        void    set(int x, int y, int z, uint8_t val);
        uint8_t get(int x, int y, int z) const;
        WTVec3  gridToWorld(int x, int y, int z) const;
    };

    // ─────────────────────────────────────────────────────────────────────────────
    //  Progress callback — called periodically so the UI can stay responsive.
    //  Return false to abort (not currently used, reserved for future use).
    // ─────────────────────────────────────────────────────────────────────────────
    using WTProgressCallback = std::function<void(int percent, const std::string& stage)>;

    // ─────────────────────────────────────────────────────────────────────────────
    //  Public API
    // ─────────────────────────────────────────────────────────────────────────────

    /// Load STL (binary or ASCII) into WTMesh.
    bool   wtLoadSTL(const std::string& path, WTMesh& mesh);

    /// Load OBJ into WTMesh.
    bool   wtLoadOBJ(const std::string& path, WTMesh& mesh);

    /// Load PLY (ASCII) into WTMesh.
    bool   wtLoadPLY(const std::string& path, WTMesh& mesh);

    /// Dispatch to the right loader based on file extension (.stl / .obj / .ply).
    bool   wtLoadMesh(const std::string& path, WTMesh& mesh);

    /// Save WTMesh as OBJ.
    bool   wtSaveOBJ(const std::string& path, const WTMesh& mesh);

    /// Save WTMesh as binary STL.
    bool   wtSaveSTL(const std::string& path, const WTMesh& mesh);

    /// Save using extension to pick format (.stl → binary STL, otherwise OBJ).
    bool   wtSaveMesh(const std::string& path, const WTMesh& mesh);

    // ── Repair steps (can be called individually) ────────────────────────────────

    /// Stage 1 – surface-based hole filling (fast, works on manifold gaps).
    void   wtFillHolesSurface(WTMesh& mesh);

    /// Stage 2a – rasterise mesh onto a voxel grid.
    void   wtRasterizeMesh(const WTMesh& mesh, WTVoxelGrid& grid);

    /// Stage 2b – flood-fill to mark outside voxels.
    void   wtFloodFillOutside(WTVoxelGrid& grid);

    /// Stage 2c – extract watertight surface from voxel boundary.
    void   wtExtractVoxelSurface(const WTVoxelGrid& grid, WTMesh& outMesh);

    /// Stage 2d – Laplacian smoothing (optional post-process).
    void   wtSmoothMesh(WTMesh& mesh, int iterations = 3);

    // ── All-in-one pipeline ───────────────────────────────────────────────────────

    /**
     * @brief Run the full two-stage watertight repair pipeline.
     *
     * Stage 1 : surface hole filling (fast, topology-preserving).
     * Stage 2 : volumetric voxel repair (handles self-intersections,
     *            non-manifold edges, large gaps).
     *
     * @param inputPath   Path to input mesh (STL / OBJ / PLY).
     * @param outputPath  Path to write the repaired mesh.
     * @param voxelRes    Voxel grid resolution (default 128; raise for finer detail).
     * @param progress    Optional progress callback.
     * @return            Empty string on success, error description on failure.
     */
    std::string wtRepairMesh(
        const std::string& inputPath,
        const std::string& outputPath,
        int                voxelRes = 128,
        WTProgressCallback progress = nullptr
    );

} // namespace Mayo