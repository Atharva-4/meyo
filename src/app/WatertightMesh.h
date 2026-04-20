#pragma once
// ============================================================
//  WatertightMesh.h  — Mayo-compatible wrapper
//  Wraps mesh_repair logic into Mayo namespace.
//  No Windows dialogs — Mayo handles file paths itself.
// ============================================================
#include <string>
#include <functional>

namespace Mayo {

using WTProgressCallback = std::function<void(int, const std::string&)>;

// Two-stage repair: surface hole fill + voxel reconstruction
// Returns empty string on success, error message on failure.
std::string wtRepairMesh(
    const std::string& inputPath,
    const std::string& outputPath,
    int  voxelResolution = 128,
    WTProgressCallback cb = nullptr
);

} // namespace Mayo
