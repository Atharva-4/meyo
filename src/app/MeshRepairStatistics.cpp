#include "MeshRepairStatistics.h"

#include "StlHoleFilling.h"

#include <CGAL/Polygon_mesh_processing/self_intersections.h>

#include <algorithm>
#include <array>
#include <map>
#include <unordered_map>
#include <vector>

namespace PMP = CGAL::Polygon_mesh_processing;

namespace Mayo {

    namespace {

        using Edge = std::pair<int, int>;

        inline Edge makeEdge(int a, int b)
        {
            if (a > b)
                std::swap(a, b);
            return { a, b };
        }

    } // namespace

    std::optional<MeshRepairStats> computeMeshRepairStatsFromStl(
        const std::string& stlFilepath,
        std::string* errorMessage)
    {
        std::vector<Triangles> triangles;
        if (isBinarySTL(stlFilepath))
            readBinarySTL(stlFilepath, triangles);
        else
            readASCIISTL(stlFilepath, triangles);

        if (triangles.empty()) {
            if (errorMessage)
                *errorMessage = "No triangles were read from STL file";
            return std::nullopt;
        }

        MeshRepairStats stats;
        stats.triangleCount = static_cast<int>(triangles.size());

        std::unordered_map<vect, int> mapVertexIndex;
        mapVertexIndex.reserve(triangles.size() * 3);

        std::vector<std::array<int, 3>> triIndices;
        triIndices.reserve(triangles.size());

        auto fnGetVertexIndex = [&](const vect& v) {
            auto it = mapVertexIndex.find(v);
            if (it != mapVertexIndex.end())
                return it->second;

            const int idx = static_cast<int>(mapVertexIndex.size());
            mapVertexIndex.insert({ v, idx });
            return idx;
            };

        for (const Triangles& tri : triangles) {
            const int i1 = fnGetVertexIndex(tri.v1);
            const int i2 = fnGetVertexIndex(tri.v2);
            const int i3 = fnGetVertexIndex(tri.v3);
            triIndices.push_back({ i1, i2, i3 });
        }

        stats.vertexCount = static_cast<int>(mapVertexIndex.size());
        stats.duplicateVertices = stats.triangleCount * 3 - stats.vertexCount;

        std::map<Edge, int> mapEdgeUseCount;
        for (const auto& tri : triIndices) {
            mapEdgeUseCount[makeEdge(tri[0], tri[1])] += 1;
            mapEdgeUseCount[makeEdge(tri[1], tri[2])] += 1;
            mapEdgeUseCount[makeEdge(tri[2], tri[0])] += 1;
        }

        stats.nonManifoldEdges = 0;
        for (const auto& pair : mapEdgeUseCount) {
            if (pair.second > 2)
                ++stats.nonManifoldEdges;
        }

        SurfaceMesh mesh = convertToSurfaceMesh(triangles);
        stats.holeCount = static_cast<int>(countHolesCGAL(mesh));

        std::vector<std::pair<SurfaceMesh::Face_index, SurfaceMesh::Face_index>> selfIntersections;
        PMP::self_intersections(mesh, std::back_inserter(selfIntersections));
        stats.selfIntersectionPairs = static_cast<int>(selfIntersections.size());

        return stats;
    }

} // namespace Mayo