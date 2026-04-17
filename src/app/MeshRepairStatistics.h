#pragma once

#include <optional>
#include <string>

namespace Mayo {

    struct MeshRepairStats {
        std::string inputFormat;
        int vertexCount = 0;
        int triangleCount = 0;
        int holeCount = 0;
        int duplicateVertices = 0;
        int nonManifoldEdges = 0;
        int selfIntersectionPairs = 0;
        int voxelResolution = 0;
        int voxelOccupiedCells = 0;
    };

    std::optional<MeshRepairStats> computeMeshRepairStatsFromMeshFile(
        const std::string& meshFilepath,
        std::string* errorMessage = nullptr);

} // namespace Mayo