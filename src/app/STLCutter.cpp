#include "STLCutter.h"
#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <cstdint>
#include <cctype>
#include <cstring>

namespace Mayo {
    namespace {

        Vec3 cross(const Vec3& a, const Vec3& b)
        {
            return {
                a.y * b.z - a.z * b.y,
                a.z * b.x - a.x * b.z,
                a.x * b.y - a.y * b.x
            };
        }

        float dot(const Vec3& a, const Vec3& b)
        {
            return a.x * b.x + a.y * b.y + a.z * b.z;
        }

        Vec3 normalize(const Vec3& v)
        {
            const float n = std::sqrt(dot(v, v));
            if (n < 1e-12f)
                return { 0.f, 0.f, 1.f };

            return { v.x / n, v.y / n, v.z / n };
        }

        Facet makeOrientedFacet(const Vec3& hintNormal, const Vec3& v1, const Vec3& v2, const Vec3& v3)
        {
            Vec3 a = v1;
            Vec3 b = v2;
            Vec3 c = v3;

            Vec3 triNormal = cross(b - a, c - a);
            if (dot(triNormal, hintNormal) < 0.f) {
                std::swap(b, c);
                triNormal = cross(b - a, c - a);
            }

            return { normalize(triNormal), a, b, c };
        }

    }
    STLCutter::STLCutter() {}

    std::vector<Facet> STLCutter::loadSTL(const std::string& filename) {
        std::vector<Facet> facets;

        std::ifstream in(filename, std::ios::binary);
        if (!in.is_open())
            return facets;

        in.seekg(0, std::ios::end);
        const std::streamoff size = in.tellg();
        in.seekg(0, std::ios::beg);
        if (size <= 0)
            return facets;

        const auto byteCount = static_cast<std::size_t>(size);
        std::vector<char> bytes(byteCount);
        in.read(bytes.data(), static_cast<std::streamsize>(byteCount));
        if (!in)
            return facets;

        bool isBinary = false;
        if (bytes.size() >= 84) {
            uint32_t triCount = 0;
            std::memcpy(&triCount, bytes.data() + 80, sizeof(uint32_t));
            const size_t expectedSize = 84u + static_cast<size_t>(triCount) * 50u;
            if (triCount > 0 && expectedSize == bytes.size()) {
                const bool startsWithSolid = std::equal(
                    bytes.begin(), bytes.begin() + 5,
                    std::string("solid").begin(),
                    [](char lhs, char rhs) {
                        return std::tolower(static_cast<unsigned char>(lhs)) == rhs;
                    }
                );

                if (!startsWithSolid) {
                    isBinary = true;
                }
                else {
                    // Binary STL may also start with "solid" in the header.
                    // If we find a null-byte early, we treat it as binary.
                    const size_t scanLimit = std::min<size_t>(bytes.size(), 256);
                    isBinary = std::find(bytes.begin(), bytes.begin() + scanLimit, '\0')
                        != bytes.begin() + scanLimit;
                }
            }
        }

        if (isBinary) {
            uint32_t triCount = 0;
            std::memcpy(&triCount, bytes.data() + 80, sizeof(uint32_t));

            facets.reserve(triCount);
            size_t offset = 84;
            for (uint32_t i = 0; i < triCount && offset + 50 <= bytes.size(); ++i, offset += 50) {
                Facet f;
                std::memcpy(&f.normal.x, bytes.data() + offset + 0, 4);
                std::memcpy(&f.normal.y, bytes.data() + offset + 4, 4);
                std::memcpy(&f.normal.z, bytes.data() + offset + 8, 4);
                std::memcpy(&f.v1.x, bytes.data() + offset + 12, 4);
                std::memcpy(&f.v1.y, bytes.data() + offset + 16, 4);
                std::memcpy(&f.v1.z, bytes.data() + offset + 20, 4);
                std::memcpy(&f.v2.x, bytes.data() + offset + 24, 4);
                std::memcpy(&f.v2.y, bytes.data() + offset + 28, 4);
                std::memcpy(&f.v2.z, bytes.data() + offset + 32, 4);
                std::memcpy(&f.v3.x, bytes.data() + offset + 36, 4);
                std::memcpy(&f.v3.y, bytes.data() + offset + 40, 4);
                std::memcpy(&f.v3.z, bytes.data() + offset + 44, 4);
                facets.push_back(f);
            }

            return facets;
        }

        std::istringstream stream(std::string(bytes.begin(), bytes.end()));
        std::string line;
        Facet facet;
        int vertex_count = 0;

        while (std::getline(stream, line)) {
            std::istringstream ls(line);
            std::string word;
            ls >> word;

            if (word == "facet") {
                std::string normal;
                ls >> normal >> facet.normal.x >> facet.normal.y >> facet.normal.z;
            }
            else if (word == "vertex") {
                Vec3 v{};
                ls >> v.x >> v.y >> v.z;
                if (vertex_count == 0) facet.v1 = v;
                else if (vertex_count == 1) facet.v2 = v;
                else if (vertex_count == 2) facet.v3 = v;
                vertex_count++;
            }
            else if (word == "endfacet") {
                if (vertex_count == 3)
                    facets.push_back(facet);

                facet = Facet();
                vertex_count = 0;
            }
        }

        return facets;
    }

    void STLCutter::saveSTL(const std::string& filename, const std::vector<Facet>& facets) {
        std::ofstream out(filename);
        out << "solid cut\n";
        for (const auto& f : facets) {
            out << "  facet normal " << f.normal.x << " " << f.normal.y << " " << f.normal.z << "\n";
            out << "    outer loop\n";
            out << "      vertex " << f.v1.x << " " << f.v1.y << " " << f.v1.z << "\n";
            out << "      vertex " << f.v2.x << " " << f.v2.y << " " << f.v2.z << "\n";
            out << "      vertex " << f.v3.x << " " << f.v3.y << " " << f.v3.z << "\n";
            out << "    endloop\n";
            out << "  endfacet\n";
        }
        out << "endsolid cut\n";
    }

    void STLCutter::getBounds(const std::vector<Facet>& facets, char axis, float& minVal, float& maxVal) {
        minVal = std::numeric_limits<float>::infinity();
        maxVal = -std::numeric_limits<float>::infinity();

        for (const auto& f : facets) {
            if (axis == 'X' || axis == 'x') {
                minVal = std::min({ minVal, f.v1.x, f.v2.x, f.v3.x });
                maxVal = std::max({ maxVal, f.v1.x, f.v2.x, f.v3.x });
            }
            else if (axis == 'Y' || axis == 'y') {
                minVal = std::min({ minVal, f.v1.y, f.v2.y, f.v3.y });
                maxVal = std::max({ maxVal, f.v1.y, f.v2.y, f.v3.y });
            }
            else if (axis == 'Z' || axis == 'z') {
                minVal = std::min({ minVal, f.v1.z, f.v2.z, f.v3.z });
                maxVal = std::max({ maxVal, f.v1.z, f.v2.z, f.v3.z });
            }
        }

        // if no facets, leave min/max as zeros
        if (facets.empty()) {
            minVal = maxVal = 0.f;
        }
    }

    void STLCutter::cutMesh(const std::vector<Facet>& facets, char axis, float cutValue,
        std::vector<Facet>& above, std::vector<Facet>& below) {
        float A = 0, B = 0, C = 0, D = 0;

        if (axis == 'X' || axis == 'x') {
            A = 1; D = -cutValue;
        }
        else if (axis == 'Y' || axis == 'y') {
            B = 1; D = -cutValue;
        }
        else if (axis == 'Z' || axis == 'z') {
            C = 1; D = -cutValue;
        }

        for (const auto& f : facets) {
            splitFacet(f, A, B, C, D, above, below);
        }
    }

    float STLCutter::planeValue(const Vec3& v, float A, float B, float C, float D) {
        return A * v.x + B * v.y + C * v.z + D;
    }

    Vec3 STLCutter::interpolate(const Vec3& p1, const Vec3& p2, float A, float B, float C, float D) {
        float v1 = planeValue(p1, A, B, C, D);
        float v2 = planeValue(p2, A, B, C, D);
        float denom = (v1 - v2);
        float t = (std::abs(denom) < EPS) ? 0.f : (v1 / (denom));
        return p1 + (p2 - p1) * t;
    }

    void STLCutter::splitFacet(const Facet& f, float A, float B, float C, float D,
        std::vector<Facet>& above, std::vector<Facet>& below) {
        std::vector<Vec3> pos, neg;

        auto test = [&](const Vec3& v) {
            if (planeValue(v, A, B, C, D) >= 0) pos.push_back(v);
            else neg.push_back(v);
            };

        test(f.v1); test(f.v2); test(f.v3);

        if (pos.size() == 3) {
            above.push_back(f);
        }
        else if (neg.size() == 3) {
            below.push_back(f);
        }
        else if (pos.size() == 2 && neg.size() == 1) {
            Vec3 p1 = pos[0], p2 = pos[1], n1 = neg[0];
            Vec3 ip1 = interpolate(n1, p1, A, B, C, D);
            Vec3 ip2 = interpolate(n1, p2, A, B, C, D);
            above.push_back(makeOrientedFacet(f.normal, p1, p2, ip2));
            above.push_back(makeOrientedFacet(f.normal, p1, ip2, ip1));
            below.push_back(makeOrientedFacet(f.normal, n1, ip1, ip2));
        }
        else if (pos.size() == 1 && neg.size() == 2) {
            Vec3 p1 = pos[0], n1 = neg[0], n2 = neg[1];
            Vec3 ip1 = interpolate(p1, n1, A, B, C, D);
            Vec3 ip2 = interpolate(p1, n2, A, B, C, D);
            below.push_back(makeOrientedFacet(f.normal, n1, n2, ip2));
            below.push_back(makeOrientedFacet(f.normal, n1, ip2, ip1));
            above.push_back(makeOrientedFacet(f.normal, p1, ip1, ip2));
        }
    }

} // namespace Mayo