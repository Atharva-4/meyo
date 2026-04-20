#pragma once

// TriToQuad.h
// Thin wrapper — the actual algorithm lives in QuadRemesher.h/.cpp
// Keep this header so existing Mayo command registration still compiles.

#include "QuadRemesher.h"

namespace Mayo {
    // Alias so old call sites still work
    using T2QProgressCallback = QRProgressCallback;

    inline std::string triToQuadConvert(
        const std::string& inputPath,
        const std::string& outputPath,
        int  targetFaceCount = 2000,
        int  smoothIter = 50,
        T2QProgressCallback cb = nullptr)
    {
        return quadRemesh(inputPath, outputPath, targetFaceCount, smoothIter, cb);
    }
}