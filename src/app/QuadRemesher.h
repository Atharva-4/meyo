#pragma once

// ============================================================
//  QuadRemesher.h
//  Instant-Meshes-style quad remeshing for Mayo.
//
//  Pipeline:
//    1. Load mesh (STL / OBJ / PLY)
//    2. Weld vertices + build half-edge connectivity
//    3. Compute per-vertex 4-RoSy orientation field
//       (smoothed with Gauss-Seidel iterations)
//    4. Integrate the RoSy field into a 2-D position field
//       (one scalar per vertex per axis)
//    5. Snap iso-lines of the position field to extract quads
//    6. Save result as OBJ (pure quad or quad-dominant)
// ============================================================

#include <functional>
#include <string>

namespace Mayo {

    /// Progress callback: (percent 0-100, status message)
    using QRProgressCallback = std::function<void(int, const std::string&)>;

    /**
     * @brief Run Instant-Meshes-style quad remeshing.
     *
     * @param inputPath       Source mesh file (STL / OBJ / PLY).
     * @param outputPath      Output OBJ file path.
     * @param targetFaceCount Desired number of quad faces in the output.
     *                        Controls the resolution of the extracted grid.
     * @param smoothIter      RoSy field smoothing iterations (default 50).
     *                        More iterations → cleaner field → better quads.
     * @param cb              Optional progress callback.
     * @return Empty string on success, error message on failure.
     */
    std::string quadRemesh(
        const std::string& inputPath,
        const std::string& outputPath,
        int  targetFaceCount = 2000,
        int  smoothIter = 50,
        QRProgressCallback cb = nullptr
    );

} // namespace Mayo