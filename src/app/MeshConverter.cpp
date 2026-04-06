#include "MeshConverter.h"

// ─── Global Mesh Data ──────────────────────────────────────────────────────

vector<Vertex> vertices;
vector<Face>   faces;

// ─── Utility ───────────────────────────────────────────────────────────────

bool fileExists(const string& name) {
    ifstream f(name.c_str());
    return f.good();
}

string trim(const string& s) {
    size_t first = s.find_first_not_of(" \t\n\r");
    if (string::npos == first) return "";
    size_t last = s.find_last_not_of(" \t\n\r");
    return s.substr(first, (last - first + 1));
}

// ─── PLY Reader ────────────────────────────────────────────────────────────

bool readPLY(string filename)
{
    vertices.clear();
    faces.clear();

    ifstream file(filename, ios::binary);
    if (!file) {
        cerr << "Error: Could not open file '" << filename << "'. Check if the file exists and path is correct.\n";
        return false;
    }

    string line;
    getline(file, line);
    if (!line.empty() && line.back() == '\r') line.pop_back();

    if (line != "ply") {
        cerr << "Error: File '" << filename << "' is not a valid PLY file (missing 'ply' header).\n";
        if (line.find("solid") != string::npos)
            cerr << "Hint: This looks like an ASCII STL file. Try converting STL -> PLY instead.\n";
        return false;
    }

    bool isBinary   = false;
    int  vertexCount = 0;
    int  faceCount   = 0;

    while (getline(file, line))
    {
        if (!line.empty() && line.back() == '\r') line.pop_back();

        stringstream ss(line);
        string keyword;
        ss >> keyword;

        if (keyword == "format") {
            string format;
            ss >> format;
            if (format == "binary_little_endian") isBinary = true;
        }
        else if (keyword == "element") {
            string type;
            ss >> type;
            if      (type == "vertex") ss >> vertexCount;
            else if (type == "face")   ss >> faceCount;
        }
        else if (keyword == "end_header") {
            break;
        }
    }

    cout << "Reading " << (isBinary ? "Binary" : "ASCII") << " PLY: "
         << vertexCount << " vertices, " << faceCount << " faces.\n";

    if (isBinary)
    {
        for (int i = 0; i < vertexCount; ++i) {
            Vertex v;
            file.read(reinterpret_cast<char*>(&v.x), sizeof(float));
            file.read(reinterpret_cast<char*>(&v.y), sizeof(float));
            file.read(reinterpret_cast<char*>(&v.z), sizeof(float));
            vertices.push_back(v);
        }

        for (int i = 0; i < faceCount; ++i) {
            unsigned char count;
            file.read(reinterpret_cast<char*>(&count), sizeof(unsigned char));
            Face f;
            for (int j = 0; j < count; ++j) {
                int idx;
                file.read(reinterpret_cast<char*>(&idx), sizeof(int));
                f.indices.push_back(idx);
            }
            faces.push_back(f);
        }
    }
    else
    {
        for (int i = 0; i < vertexCount; ++i) {
            Vertex v;
            file >> v.x >> v.y >> v.z;
            string dummy;
            getline(file, dummy);
            vertices.push_back(v);
        }

        for (int i = 0; i < faceCount; ++i) {
            int count;
            file >> count;
            Face f;
            for (int j = 0; j < count; ++j) {
                int idx;
                file >> idx;
                f.indices.push_back(idx);
            }
            faces.push_back(f);
        }
    }

    return true;
}

// ─── STL Reader ────────────────────────────────────────────────────────────

bool readSTL(string filename)
{
    vertices.clear();
    faces.clear();

    ifstream file(filename, ios::binary | ios::ate);
    if (!file) {
        cerr << "Error: Could not open file " << filename << endl;
        return false;
    }

    streamsize size = file.tellg();
    file.seekg(0, ios::beg);

    unsigned int count = 0;

    if (size >= 84)
    {
        file.seekg(80, ios::beg);
        file.read(reinterpret_cast<char*>(&count), 4);

        unsigned long long expectedSize = 84 + (unsigned long long)count * 50;

        if ((unsigned long long)size >= expectedSize && count > 0 &&
            (unsigned long long)(size - 84) / 50 >= count)
        {
            cout << "Detected Binary STL with " << count << " triangles.\n";

            file.seekg(84, ios::beg);
            for (unsigned int i = 0; i < count; ++i)
            {
                float n[3], v1[3], v2[3], v3[3];
                unsigned short attr;

                if (!file.read(reinterpret_cast<char*>(n),   12)) break;
                if (!file.read(reinterpret_cast<char*>(v1),  12)) break;
                if (!file.read(reinterpret_cast<char*>(v2),  12)) break;
                if (!file.read(reinterpret_cast<char*>(v3),  12)) break;
                if (!file.read(reinterpret_cast<char*>(&attr), 2)) break;

                int base = static_cast<int>(vertices.size());
                vertices.push_back({ v1[0], v1[1], v1[2] });
                vertices.push_back({ v2[0], v2[1], v2[2] });
                vertices.push_back({ v3[0], v3[1], v3[2] });

                faces.push_back({ {base, base + 1, base + 2} });
            }

            cout << "Successfully read " << faces.size() << " facets from Binary STL.\n";
            return true;
        }
    }

    // ── ASCII fallback ────────────────────────────────────────────────────
    cout << "Detected ASCII STL.\n";
    file.close();
    file.open(filename);

    string line;
    while (getline(file, line))
    {
        string trimmed = trim(line);
        if (trimmed.find("vertex") != string::npos ||
            trimmed.find("VERTEX") != string::npos)
        {
            stringstream ss(trimmed);
            string key;
            float x, y, z;
            ss >> key >> x >> y >> z;
            vertices.push_back({ x, y, z });

            if (vertices.size() % 3 == 0) {
                int base = static_cast<int>(vertices.size() - 3);
                faces.push_back({ {base, base + 1, base + 2} });
            }
        }
    }

    if (faces.empty()) {
        cerr << "Error: Could not find any valid triangles in STL file.\n";
        return false;
    }

    cout << "Successfully read " << faces.size() << " facets from ASCII STL.\n";
    return true;
}

// ─── OBJ Reader ────────────────────────────────────────────────────────────

bool readOBJ(string filename)
{
    vertices.clear();
    faces.clear();

    ifstream file(filename);
    if (!file) {
        cerr << "Error: Could not open file " << filename << endl;
        return false;
    }

    string line;
    while (getline(file, line))
    {
        if (line.substr(0, 2) == "v ")
        {
            stringstream ss(line.substr(2));
            Vertex v = { 0, 0, 0 };
            ss >> v.x >> v.y >> v.z;
            vertices.push_back(v);
        }
        else if (line.substr(0, 2) == "f ")
        {
            stringstream ss(line.substr(2));
            string segment;
            Face f;
            while (ss >> segment)
            {
                size_t slashPos = segment.find('/');
                if (slashPos != string::npos)
                    segment = segment.substr(0, slashPos);
                f.indices.push_back(stoi(segment) - 1);
            }
            faces.push_back(f);
        }
    }

    cout << "Read OBJ: " << vertices.size() << " vertices, " << faces.size() << " faces.\n";
    return true;
}

// ─── PLY Writer ────────────────────────────────────────────────────────────

bool writePLY(string filename)
{
    ofstream file(filename);
    if (!file) {
        cerr << "Error: Could not create file " << filename << endl;
        return false;
    }

    file << "ply\n"
         << "format ascii 1.0\n"
         << "element vertex " << vertices.size() << "\n"
         << "property float x\n"
         << "property float y\n"
         << "property float z\n"
         << "element face " << faces.size() << "\n"
         << "property list uchar int vertex_indices\n"
         << "end_header\n";

    for (const auto& v : vertices)
        file << v.x << " " << v.y << " " << v.z << "\n";

    for (const auto& f : faces) {
        file << f.indices.size();
        for (int idx : f.indices) file << " " << idx;
        file << "\n";
    }

    cout << "Written PLY to " << filename << endl;
    return true;
}

// ─── STL Binary Writer ─────────────────────────────────────────────────────

bool writeSTLBinary(string filename)
{
    ofstream file(filename, ios::binary);
    if (!file) {
        cerr << "Error: Could not create output file " << filename << endl;
        return false;
    }

    char header[80] = "Binary STL file created by MeshConverter Ultimate";
    file.write(header, 80);

    unsigned int totalTriangles = 0;
    for (const auto& f : faces)
        if (f.indices.size() >= 3)
            totalTriangles += static_cast<unsigned int>(f.indices.size() - 2);

    file.write(reinterpret_cast<const char*>(&totalTriangles), 4);

    for (const auto& f : faces)
    {
        if (f.indices.size() < 3) continue;

        for (size_t i = 1; i < f.indices.size() - 1; ++i)
        {
            int i0 = f.indices[0];
            int i1 = f.indices[i];
            int i2 = f.indices[i + 1];

            if (i0 >= (int)vertices.size() ||
                i1 >= (int)vertices.size() ||
                i2 >= (int)vertices.size()) continue;

            Vertex v1 = vertices[i0];
            Vertex v2 = vertices[i1];
            Vertex v3 = vertices[i2];

            float ux = v2.x - v1.x, uy = v2.y - v1.y, uz = v2.z - v1.z;
            float vx = v3.x - v1.x, vy = v3.y - v1.y, vz = v3.z - v1.z;

            float nx = uy * vz - uz * vy;
            float ny = uz * vx - ux * vz;
            float nz = ux * vy - uy * vx;
            float len = sqrt(nx * nx + ny * ny + nz * nz);
            if (len > 0) { nx /= len; ny /= len; nz /= len; }

            float normal[3] = { nx, ny, nz };
            float ver1[3]   = { v1.x, v1.y, v1.z };
            float ver2[3]   = { v2.x, v2.y, v2.z };
            float ver3[3]   = { v3.x, v3.y, v3.z };
            unsigned short attr = 0;

            file.write(reinterpret_cast<const char*>(normal), 12);
            file.write(reinterpret_cast<const char*>(ver1),   12);
            file.write(reinterpret_cast<const char*>(ver2),   12);
            file.write(reinterpret_cast<const char*>(ver3),   12);
            file.write(reinterpret_cast<const char*>(&attr),   2);
        }
    }

    cout << "Written Binary STL to " << filename
         << " (" << totalTriangles << " triangles)" << endl;
    return true;
}

// ─── STL ASCII Writer ──────────────────────────────────────────────────────

bool writeSTLAscii(string filename)
{
    ofstream file(filename);
    if (!file) {
        cerr << "Error: Could not create output file " << filename << endl;
        return false;
    }

    file << "solid mesh\n";

    for (const auto& f : faces)
    {
        if (f.indices.size() < 3) continue;

        for (size_t i = 1; i < f.indices.size() - 1; ++i)
        {
            int i0 = f.indices[0];
            int i1 = f.indices[i];
            int i2 = f.indices[i + 1];

            if (i0 >= (int)vertices.size() ||
                i1 >= (int)vertices.size() ||
                i2 >= (int)vertices.size()) continue;

            Vertex v1 = vertices[i0];
            Vertex v2 = vertices[i1];
            Vertex v3 = vertices[i2];

            float ux = v2.x - v1.x, uy = v2.y - v1.y, uz = v2.z - v1.z;
            float vx = v3.x - v1.x, vy = v3.y - v1.y, vz = v3.z - v1.z;

            float nx = uy * vz - uz * vy;
            float ny = uz * vx - ux * vz;
            float nz = ux * vy - uy * vx;
            float len = sqrt(nx * nx + ny * ny + nz * nz);
            if (len > 0) { nx /= len; ny /= len; nz /= len; }

            file << "  facet normal " << nx << " " << ny << " " << nz << "\n"
                 << "    outer loop\n"
                 << "      vertex " << v1.x << " " << v1.y << " " << v1.z << "\n"
                 << "      vertex " << v2.x << " " << v2.y << " " << v2.z << "\n"
                 << "      vertex " << v3.x << " " << v3.y << " " << v3.z << "\n"
                 << "    endloop\n"
                 << "  endfacet\n";
        }
    }

    file << "endsolid mesh\n";
    cout << "Written ASCII STL to " << filename << endl;
    return true;
}

// ─── OBJ Writer ────────────────────────────────────────────────────────────

bool writeOBJ(string filename)
{
    ofstream file(filename);
    if (!file) {
        cerr << "Error: Could not create file " << filename << endl;
        return false;
    }

    file << "# Converted by MeshConverter\n";

    for (const auto& v : vertices)
        file << "v " << v.x << " " << v.y << " " << v.z << "\n";

    for (const auto& f : faces) {
        file << "f";
        for (int idx : f.indices)
            file << " " << (idx + 1);
        file << "\n";
    }

    cout << "Written OBJ to " << filename << endl;
    return true;
}
