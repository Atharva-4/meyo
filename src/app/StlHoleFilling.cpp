//#include "StlHoleFilling.h"
//#include <vector>
//#include <unordered_set>
//
//namespace Mayo {
//
//    // Return one ordered loop per hole; each loop is vector of CGAL::Point_3 (Mayo::Point)
//    std::vector<std::vector<Point>> extractHoleBoundaries(const SurfaceMesh& mesh)
//    {
//        std::vector<std::vector<Point>> result;
//
//        using Halfedge_index = SurfaceMesh::Halfedge_index;
//        using Vertex_index = SurfaceMesh::Vertex_index;
//
//        std::unordered_set<std::size_t> visited;
//
//        for (Halfedge_index h : mesh.halfedges()) {
//            if (!mesh.is_border(h))
//                continue;
//
//            const std::size_t h_idx = static_cast<std::size_t>(h);
//            if (visited.find(h_idx) != visited.end())
//                continue;
//
//            std::vector<Point> loop;
//            Halfedge_index curr = h;
//            // traverse border cycle
//            do {
//                Vertex_index v = mesh.target(curr);
//                loop.push_back(mesh.point(v));
//                visited.insert(static_cast<std::size_t>(curr));
//                curr = mesh.next(curr);
//            } while (curr != h && curr != SurfaceMesh::null_halfedge());
//
//            if (loop.size() >= 2)
//                result.push_back(std::move(loop));
//        }
//
//        return result;
//    }
//
//} // namespace Mayo