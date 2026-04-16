#pragma once

#include <optional>
#include <string>

namespace Mayo {

    struct MeshRepairStats {
        int vertexCount = 0;
        int triangleCount = 0;
        int holeCount = 0;
        int duplicateVertices = 0;
        int nonManifoldEdges = 0;
        int selfIntersectionPairs = 0;
    };

    std::optional<MeshRepairStats> computeMeshRepairStatsFromStl(
        const std::string& stlFilepath,
        std::string* errorMessage = nullptr);

} // namespace Mayo
