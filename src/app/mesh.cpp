#define _CRT_SECURE_NO_WARNINGS
// ******************************
// mesh.cpp
//
// Mesh class, which stores a list
// of vertices & a list of triangles.
//

#include <assert.h>
#include <float.h>

#if defined (_MSC_VER) && (_MSC_VER >= 1020)
#pragma warning(disable:4710) // function not inlined
#pragma warning(disable:4702) // unreachable code
#pragma warning(disable:4514) // unreferenced inline function has been removed
#endif

#include <iostream>
#include <map>
#include <vector>

#include<fstream>
#include<string>
#include<math.h>

#include "mesh.h"


Mesh::Mesh(char* filename)
{
	_numVerts = _numTriangles = 0;
	_isStlBinary = false;
	if (!loadFromFile(filename))
	{
		// we failed to load mesh from the file
		_numVerts = _numTriangles = 0;
		_vlist.clear();
		_plist.clear();
	}
}

Mesh::Mesh(const Mesh& m)
{
	_numVerts = m._numVerts;
	_numTriangles = m._numTriangles;
	_isStlBinary = m._isStlBinary;
	_vlist = m._vlist; // NOTE: triangles are still pointing to original mesh
	_plist = m._plist;
	// NOTE: should reset tris in _vlist, _plist
}

Mesh& Mesh::operator=(const Mesh& m)
{
	if (this == &m) return *this; // don't assign to self
	_numVerts = m._numVerts;
	_numTriangles = m._numTriangles;
	_isStlBinary = m._isStlBinary;
	_vlist = m._vlist; // NOTE: triangles are still pointing to original mesh
	_plist = m._plist;
	// NOTE: should reset tris in _vlist, _plist
	return *this;
}

Mesh::~Mesh()
{
	_numVerts = _numTriangles = 0;
	_vlist.erase(_vlist.begin(), _vlist.end());
	_plist.erase(_plist.begin(), _plist.end());
}


// Helper function for reading PLY mesh file
bool Mesh::readPlyHeader(FILE*& inFile)
{
	char line[1024];
	_numVerts = 0;
	_numTriangles = 0;
	_plyFormat = PLY_ASCII;
	_vertStride = 0;
	_hasTexCoords = false;
	_xIdx = _yIdx = _zIdx = _sIdx = _tIdx = -1;
	_xOff = _yOff = _zOff = _sOff = _tOff = -1;
	int currentPropIdx = 0;

	bool inHeader = true;
	bool inVertex = false;
	while (inHeader && fgets(line, sizeof(line), inFile)) {
		char cmd[256];
		if (sscanf(line, "%s", cmd) != 1) continue;
		ChangeStrToLower(cmd);

		if (strcmp(cmd, "format") == 0) {
			char fmt[256];
			sscanf(line, "%*s %s", fmt);
			if (strstr(fmt, "binary_little_endian")) _plyFormat = PLY_BINARY_LE;
			else if (strstr(fmt, "binary_big_endian")) _plyFormat = PLY_BINARY_BE;
		}
		else if (strcmp(cmd, "element") == 0) {
			char type[256];
			int count;
			sscanf(line, "%*s %s %d", type, &count);
			if (strcmp(type, "vertex") == 0) { _numVerts = count; inVertex = true; }
			else if (strcmp(type, "face") == 0 || strcmp(type, "polygon") == 0) { _numTriangles = count; inVertex = false; }
			else inVertex = false;
		}
		else if (strcmp(cmd, "property") == 0) {
			if (inVertex) {
				char type[256], propName[256];
				sscanf(line, "%*s %s %s", type, propName);
				ChangeStrToLower(propName);
				
				int bytes = 0;
				if (strcmp(type, "float") == 0 || strcmp(type, "int") == 0 || strcmp(type, "uint") == 0) bytes = 4;
				else if (strcmp(type, "char") == 0 || strcmp(type, "uchar") == 0) bytes = 1;
				else if (strcmp(type, "double") == 0) bytes = 8;
				else if (strcmp(type, "short") == 0 || strcmp(type, "ushort") == 0) bytes = 2;

				if (strcmp(propName, "x") == 0) { _xOff = _vertStride; _xIdx = currentPropIdx; }
				else if (strcmp(propName, "y") == 0) { _yOff = _vertStride; _yIdx = currentPropIdx; }
				else if (strcmp(propName, "z") == 0) { _zOff = _vertStride; _zIdx = currentPropIdx; }
				else if (strcmp(propName, "s") == 0 || strcmp(propName, "u") == 0) {
					_hasTexCoords = true;
					_sIdx = currentPropIdx;
					_sOff = _vertStride;
				}
				else if (strcmp(propName, "t") == 0 || strcmp(propName, "v") == 0) {
					_tIdx = currentPropIdx;
					_tOff = _vertStride;
				}

				_vertStride += bytes;
				currentPropIdx++;
			}
		}
		else if (strcmp(cmd, "end_header") == 0) {
			inHeader = false;
		}
	}

	if (_numVerts > 0) _vlist.reserve(_numVerts);
	if (_numTriangles > 0) _plist.reserve(_numTriangles);

	return (_numVerts > 0);
}

// Helper function for reading PLY mesh file
bool Mesh::readPlyVerts(FILE*& inFile)
{
	_vlist.clear();
	if (_plyFormat == PLY_ASCII) {
		for (int i = 0; i < _numVerts; i++) {
			char line[1024];
			if (!fgets(line, sizeof(line), inFile)) break;
			
			float vals[32];
			int n = 0;
			char* p = strtok(line, " \t\r\n");
			while (p && n < 32) {
				vals[n++] = (float)atof(p);
				p = strtok(NULL, " \t\r\n");
			}

			float x = 0, y = 0, z = 0, s = 0, t = 0;
			
			// Mapping for ASCII based on property order detected in header
			int xIdx = (_xIdx != -1) ? _xIdx : 0;
			int yIdx = (_yIdx != -1) ? _yIdx : 1;
			int zIdx = (_zIdx != -1) ? _zIdx : 2;
			
			x = (xIdx < n) ? vals[xIdx] : 0;
			y = (yIdx < n) ? vals[yIdx] : 0;
			z = (zIdx < n) ? vals[zIdx] : 0;
			
			vertex v(x, y, z);
			if (_hasTexCoords) {
				if (_sIdx != -1 && _sIdx < n) s = vals[_sIdx];
				if (_tIdx != -1 && _tIdx < n) t = vals[_tIdx];
				v.setTexCoords(s, t);
			}
			
			v.setIndex((int)_vlist.size());
			_vlist.push_back(v);
		}
	} else {
		// Binary
		char* buffer = new char[_vertStride];
		for (int i = 0; i < _numVerts; i++) {
			if (fread(buffer, _vertStride, 1, inFile) != 1) break;
			
			auto getFloat = [&](int off) -> float {
				if (off < 0 || off + 4 > _vertStride) return 0.0f;
				float f;
				memcpy(&f, buffer + off, 4);
				if (_plyFormat == PLY_BINARY_BE) {
					unsigned char* p = (unsigned char*)&f;
					unsigned char tmp;
					tmp = p[0]; p[0] = p[3]; p[3] = tmp;
					tmp = p[1]; p[1] = p[2]; p[2] = tmp;
				}
				return f;
			};

			float x = (_xOff != -1) ? getFloat(_xOff) : 0;
			float y = (_yOff != -1) ? getFloat(_yOff) : 0;
			float z = (_zOff != -1) ? getFloat(_zOff) : 0;
			
			vertex vert(x, y, z);
			if (_hasTexCoords) {
				float s = (_sOff != -1) ? getFloat(_sOff) : 0;
				float t = (_tOff != -1) ? getFloat(_tOff) : 0;
				vert.setTexCoords(s, t);
			}
			vert.setIndex((int)_vlist.size());
			_vlist.push_back(vert);
		}
		delete[] buffer;
	}
	_numVerts = (int)_vlist.size();
	return true;
}

// Helper function for reading PLY mesh file
bool Mesh::readPlyTris(FILE*& inFile)
{
	_plist.clear();
	int actualTris = 0;
	if (_plyFormat == PLY_ASCII) {
		for (int i = 0; i < _numTriangles; i++) {
			int nVerts;
			if (fscanf(inFile, "%d", &nVerts) != 1) break;
			if (nVerts == 3) {
				int v1, v2, v3;
				if (fscanf(inFile, "%d %d %d", &v1, &v2, &v3) == 3) {
					if (v1 >= 0 && v1 < _numVerts && v2 >= 0 && v2 < _numVerts && v3 >= 0 && v3 < _numVerts) {
						triangle t(this, v1, v2, v3);
						t.setIndex(actualTris);
						_plist.push_back(t);
						_vlist[v1].addTriNeighbor(actualTris);
						_vlist[v1].addVertNeighbor(v2); _vlist[v1].addVertNeighbor(v3);
						_vlist[v2].addTriNeighbor(actualTris);
						_vlist[v2].addVertNeighbor(v1); _vlist[v2].addVertNeighbor(v3);
						_vlist[v3].addTriNeighbor(actualTris);
						_vlist[v3].addVertNeighbor(v1); _vlist[v3].addVertNeighbor(v2);
						actualTris++;
					}
				}
				// Skip any additional vertex indices (for polygons > 3)
				for (int k = 3; k < nVerts; k++) { int dummy; fscanf(inFile, "%d", &dummy); }
			} else if (nVerts == 4) {
				int v[4];
				if (fscanf(inFile, "%d %d %d %d", &v[0], &v[1], &v[2], &v[3]) == 4) {
					// Split quad into two triangles: (0,1,2) and (0,2,3)
					int tris[2][3] = {{v[0], v[1], v[2]}, {v[0], v[2], v[3]}};
					for (int j = 0; j < 2; j++) {
						int v1 = tris[j][0], v2 = tris[j][1], v3 = tris[j][2];
						if (v1 >= 0 && v1 < _numVerts && v2 >= 0 && v2 < _numVerts && v3 >= 0 && v3 < _numVerts) {
							triangle t(this, v1, v2, v3);
							t.setIndex(actualTris);
							_plist.push_back(t);
							_vlist[v1].addTriNeighbor(actualTris);
							_vlist[v1].addVertNeighbor(v2); _vlist[v1].addVertNeighbor(v3);
							_vlist[v2].addTriNeighbor(actualTris);
							_vlist[v2].addVertNeighbor(v1); _vlist[v2].addVertNeighbor(v3);
							_vlist[v3].addTriNeighbor(actualTris);
							_vlist[v3].addVertNeighbor(v1); _vlist[v3].addVertNeighbor(v2);
							actualTris++;
						}
					}
				}
			} else {
				for (int k = 0; k < nVerts; k++) { int j; fscanf(inFile, "%d", &j); }
			}
			while (fgetc(inFile) != '\n' && !feof(inFile));
		}
	} else {
		// Binary
		for (int i = 0; i < _numTriangles; i++) {
			unsigned char nVerts;
			if (fread(&nVerts, 1, 1, inFile) != 1) break;
			if (nVerts == 3) {
				int v[3];
				if (fread(v, sizeof(int), 3, inFile) != 3) break;
				if (v[0] >= 0 && v[0] < _numVerts && v[1] >= 0 && v[1] < _numVerts && v[2] >= 0 && v[2] < _numVerts) {
					triangle t(this, v[0], v[1], v[2]);
					t.setIndex(actualTris);
					_plist.push_back(t);
					_vlist[v[0]].addTriNeighbor(actualTris);
					_vlist[v[0]].addVertNeighbor(v[1]); _vlist[v[0]].addVertNeighbor(v[2]);
					_vlist[v[1]].addTriNeighbor(actualTris);
					_vlist[v[1]].addVertNeighbor(v[0]); _vlist[v[1]].addVertNeighbor(v[2]);
					_vlist[v[2]].addTriNeighbor(actualTris);
					_vlist[v[2]].addVertNeighbor(v[0]); _vlist[v[2]].addVertNeighbor(v[1]);
					actualTris++;
				}
			} else if (nVerts == 4) {
				int v[4];
				if (fread(v, sizeof(int), 4, inFile) != 4) break;
				int tris[2][3] = {{v[0], v[1], v[2]}, {v[0], v[2], v[3]}};
				for (int j = 0; j < 2; j++) {
					int v1 = tris[j][0], v2 = tris[j][1], v3 = tris[j][2];
					if (v1 >= 0 && v1 < _numVerts && v2 >= 0 && v2 < _numVerts && v3 >= 0 && v3 < _numVerts) {
						triangle t(this, v1, v2, v3);
						t.setIndex(actualTris);
						_plist.push_back(t);
						_vlist[v1].addTriNeighbor(actualTris);
						_vlist[v1].addVertNeighbor(v2); _vlist[v1].addVertNeighbor(v3);
						_vlist[v2].addTriNeighbor(actualTris);
						_vlist[v2].addVertNeighbor(v1); _vlist[v2].addVertNeighbor(v3);
						_vlist[v3].addTriNeighbor(actualTris);
						_vlist[v3].addVertNeighbor(v1); _vlist[v3].addVertNeighbor(v2);
						actualTris++;
					}
				}
			} else {
				fseek(inFile, nVerts * sizeof(int), SEEK_CUR);
			}
		}
	}
	_numTriangles = (int)_plist.size();
	return true;
}


// Center mesh around origin.
// Fit mesh in box from (-1, -1, -1) to (1, 1, 1)
void Mesh::Normalize()
{
	float min[3], max[3], Scale;

	if (_vlist.empty()) return;

	setMinMax(min, max);

	Vec3 minv(min);
	Vec3 maxv(max);

	Vec3 dimv = maxv - minv;

	if (dimv.x >= dimv.y && dimv.x >= dimv.z) Scale = (dimv.x > 0) ? (2.0f / dimv.x) : 1.0f;
	else if (dimv.y >= dimv.x && dimv.y >= dimv.z) Scale = (dimv.y > 0) ? (2.0f / dimv.y) : 1.0f;
	else Scale = (dimv.z > 0) ? (2.0f / dimv.z) : 1.0f;

	Vec3 transv = minv + maxv;

	transv *= 0.5f;

	for (unsigned int i = 0; i < _vlist.size(); ++i)
	{
		_vlist[i].getXYZ() -= transv;
		_vlist[i].getXYZ() *= Scale;
	}
}

// Load mesh from PLY or STL file
bool Mesh::loadFromFile(char* filename)
{
	char ext[_MAX_EXT];
	_splitpath(filename, NULL, NULL, NULL, ext);

	bool success = false;
	if (_stricmp(ext, ".ply") == 0)
	{
		FILE* inFile = fopen(filename, "rb");
		if (inFile == NULL) return false;
		if (readPlyHeader(inFile) && readPlyVerts(inFile) && readPlyTris(inFile)) success = true;
		fclose(inFile);
	}
	else if (_stricmp(ext, ".stl") == 0)
	{
		success = loadFromSTLFile(filename);
	}
	else if (_stricmp(ext, ".obj") == 0)
	{
		success = loadFromOBJFile(filename);
	}

	if (success)
	{
		Normalize();
		calcVertNormals();
		return true;
	}
	return false;
}

bool Mesh::loadFromSTLFile(char* filename)
{
	_vlist.clear();
	_plist.clear();

	FILE* fp = fopen(filename, "rb");
	if (!fp) return false;

	// Identify if it's a binary STL
	fseek(fp, 0, SEEK_END);
	long fileSize = ftell(fp);
	fseek(fp, 0, SEEK_SET);

	bool bBinary = false;
	unsigned int numTris = 0;

	if (fileSize >= 84) {
		char header[80];
		if (fread(header, 1, 80, fp) == 80) {
			if (fread(&numTris, 4, 1, fp) == 1) {
				if (numTris > 0 && 84 + (long)numTris * 50 == fileSize) {
					bBinary = true;
				}
			}
		}
	}

	if (bBinary) {
		// Binary STL
		_isStlBinary = true;
		_vlist.reserve(numTris); 
		_plist.reserve(numTris);

		map<pair<float, pair<float, float>>, int> vertexMap;

		for (unsigned int i = 0; i < numTris; i++) {
			float n[3], v[3][3];
			unsigned short attr;
			if (fread(n, 4, 3, fp) != 3) break;
			if (fread(v, 4, 9, fp) != 9) break;
			if (fread(&attr, 2, 1, fp) != 1) break;

			int vIndices[3];
			for (int k = 0; k < 3; k++) {
				auto vKey = make_pair(v[k][0], make_pair(v[k][1], v[k][2]));
				map<pair<float, pair<float, float>>, int>::iterator it = vertexMap.find(vKey);
				if (it == vertexMap.end()) {
					int idx = (int)_vlist.size();
					vIndices[k] = idx;
					vertexMap[vKey] = idx;
					vertex meshV(v[k][0], v[k][1], v[k][2]);
					meshV.setIndex(idx);
					_vlist.push_back(meshV);
				} else {
					vIndices[k] = it->second;
				}
			}

			triangle t(this, vIndices[0], vIndices[1], vIndices[2]);
			t.setIndex(i);
			_plist.push_back(t);

			for (int k = 0; k < 3; k++) {
				_vlist[vIndices[k]].addTriNeighbor(i);
				_vlist[vIndices[k]].addVertNeighbor(vIndices[(k + 1) % 3]);
				_vlist[vIndices[k]].addVertNeighbor(vIndices[(k + 2) % 3]);
			}
		}
		fclose(fp);
		_numVerts = (int)_vlist.size();
		_numTriangles = (int)_plist.size();
		return (_numTriangles > 0);
	} else {
		// ASCII STL
		_isStlBinary = false;
		fclose(fp);
		ifstream file(filename);
		if (!file.is_open()) return false;

		string line;
		map<pair<float, pair<float, float>>, int> vertexMap;
		int triCount = 0;

		_vlist.reserve(1000); 
		_plist.reserve(2000);

		while (getline(file, line)) {
			string checkLine = line;
			for(int ci=0; ci < (int)checkLine.length(); ci++) checkLine[ci] = (char)tolower(checkLine[ci]);

			if (checkLine.find("facet normal") != string::npos) {
				float nx, ny, nz;
				// Still use original line for sscanf as it's just reading floats
				if (sscanf(line.c_str(), "%*s %*s %f %f %f", &nx, &ny, &nz) != 3) continue;
				
				int vIndices[3];
				bool bValidTri = true;
				for (int i = 0; i < 3; ++i) {
					while (getline(file, line)) {
						string vCheck = line;
						for(int vi=0; vi < (int)vCheck.length(); vi++) vCheck[vi] = (char)tolower(vCheck[vi]);
						if (vCheck.find("vertex") != string::npos) break;
					}
					float vx, vy, vz;
					if (sscanf(line.c_str(), "%*s %f %f %f", &vx, &vy, &vz) != 3) {
						bValidTri = false;
						break;
					}
					
					auto vKey = make_pair(vx, make_pair(vy, vz));
					map<pair<float, pair<float, float>>, int>::iterator it = vertexMap.find(vKey);
					if (it == vertexMap.end()) {
						int idx = (int)_vlist.size();
						vertexMap[vKey] = idx;
						vertex v(vx, vy, vz);
						v.setIndex(idx);
						_vlist.push_back(v);
						vIndices[i] = idx;
					} else {
						vIndices[i] = it->second;
					}
				}

				if (bValidTri) {
					triangle t(this, vIndices[0], vIndices[1], vIndices[2]);
					t.setIndex(triCount);
					_plist.push_back(t);

					_vlist[vIndices[0]].addTriNeighbor(triCount);
					_vlist[vIndices[0]].addVertNeighbor(vIndices[1]);
					_vlist[vIndices[0]].addVertNeighbor(vIndices[2]);

					_vlist[vIndices[1]].addTriNeighbor(triCount);
					_vlist[vIndices[1]].addVertNeighbor(vIndices[0]);
					_vlist[vIndices[1]].addVertNeighbor(vIndices[2]);

					_vlist[vIndices[2]].addTriNeighbor(triCount);
					_vlist[vIndices[2]].addVertNeighbor(vIndices[0]);
					_vlist[vIndices[2]].addVertNeighbor(vIndices[1]);

					triCount++;
				}
			}
		}
		_numVerts = (int)_vlist.size();
		_numTriangles = (int)_plist.size();
		return (_numTriangles > 0);
	}
}

bool Mesh::loadFromOBJFile(char* filename)
{
	_vlist.clear();
	_plist.clear();

	// Measure file size to estimate memory needs
	struct stat st;
	if (stat(filename, &st) == 0) {
		// Heuristic: OBJ files average 50-100 bytes per face/vertex.
		// Reserving prevents expensive re-allocations of the vertex/triangle vectors.
		// For a 100MB file, this might reserve ~1M elements.
		long long estimatedElements = st.st_size / 60; 
		if (estimatedElements > 1000000) estimatedElements = 1000000; // Cap at 1M to be safe
		if (estimatedElements > 0) {
			_vlist.reserve((size_t)estimatedElements);
			_plist.reserve((size_t)estimatedElements * 2);
		}
	}

	FILE* fp = fopen(filename, "r");
	if (!fp) return false;

	char line[1024];
	while (fgets(line, sizeof(line), fp)) {
		char type[10] = { 0 };
		if (sscanf(line, "%9s", type) != 1) continue;

		if (strcmp(type, "v") == 0) {
			float x, y, z;
			if (sscanf(line, "%*s %f %f %f", &x, &y, &z) >= 3) {
				vertex v(x, y, z);
				v.setIndex((int)_vlist.size());
				_vlist.push_back(v);
			}
		}
		else if (strcmp(type, "f") == 0) {
			int vIndices[4];
			int n = 0;
			// Find the first space/tab after 'f'
			char* pSearch = line;
			while (*pSearch && !isspace(*pSearch)) pSearch++;
			char* p = strtok(pSearch, " \t\r\n");
			while (p && n < 4) {
				int vidx = 0;
				if (sscanf(p, "%d", &vidx) == 1) {
					if (vidx < 0) vidx = (int)_vlist.size() + vidx + 1;
					if (vidx > 0 && vidx <= (int)_vlist.size()) {
						vIndices[n++] = vidx - 1;
					}
				}
				p = strtok(NULL, " \t\r\n");
			}

			if (n >= 3) {
				int tris[2][3] = { { vIndices[0], vIndices[1], vIndices[2] }, { vIndices[0], vIndices[2], vIndices[3] } };
				int numTris = (n == 4) ? 2 : 1;
				for (int i = 0; i < numTris; i++) {
					int v1 = tris[i][0], v2 = tris[i][1], v3 = tris[i][2];
					int triIdx = (int)_plist.size();
					triangle t(this, v1, v2, v3);
					t.setIndex(triIdx);
					_plist.push_back(t);
					_vlist[v1].addTriNeighbor(triIdx); _vlist[v1].addVertNeighbor(v2); _vlist[v1].addVertNeighbor(v3);
					_vlist[v2].addTriNeighbor(triIdx); _vlist[v2].addVertNeighbor(v1); _vlist[v2].addVertNeighbor(v3);
					_vlist[v3].addTriNeighbor(triIdx); _vlist[v3].addVertNeighbor(v1); _vlist[v3].addVertNeighbor(v2);
				}
			}
		}
	}
	fclose(fp);
	_numVerts = (int)_vlist.size();
	_numTriangles = (int)_plist.size();
	return (_numTriangles > 0);
}


// Recalculate the normal for one vertex
void Mesh::calcOneVertNormal(unsigned vert)
{
	vertex& v = getVertex(vert);
	const vector<int>& triset = v.getTriNeighbors();

	vector<int>::const_iterator iter;

	Vec3 vec;

	for (iter = triset.begin(); iter != triset.end(); ++iter)
	{
		// get the triangles for each vertex & add up the normals.
		vec += getTri(*iter).getNormalVec3();
	}

	vec.normalize(); // normalize the vertex	
	v.setVertNomal(vec);
}


// Calculate the vertex normals after loading the mesh.
void Mesh::calcVertNormals()
{
	// Iterate through the vertices
	int i;
	for (i = 0; i < (int)_vlist.size(); ++i) 
	{
		calcOneVertNormal(i);
	}
}


// Get min, max values of all verts
void Mesh::setMinMax(float min[3], float max[3])
{
	max[0] = max[1] = max[2] = -FLT_MAX;
	min[0] = min[1] = min[2] = FLT_MAX;

	for (unsigned int i = 0; i < _vlist.size(); ++i)
	{
		const float* pVert = _vlist[i].getArrayVerts();
		if (pVert[0] < min[0]) min[0] = pVert[0];
		if (pVert[1] < min[1]) min[1] = pVert[1];
		if (pVert[2] < min[2]) min[2] = pVert[2];
		if (pVert[0] > max[0]) max[0] = pVert[0];
		if (pVert[1] > max[1]) max[1] = pVert[1];
		if (pVert[2] > max[2]) max[2] = pVert[2];
	}
}

// Save mesh to PLY or STL file
bool Mesh::saveToFile(char* filename)
{
	char ext[_MAX_EXT];
	_splitpath(filename, NULL, NULL, NULL, ext);
	if (_stricmp(ext, ".ply") == 0) return saveToPLY(filename);
	if (_stricmp(ext, ".stl") == 0) return saveToSTL(filename);
	if (_stricmp(ext, ".obj") == 0) return saveToOBJ(filename);
	return false;
}

bool Mesh::saveToPLY(char* filename)
{
	FILE* fp = fopen(filename, "w");
	if (!fp) return false;

	vector<int> newId(_vlist.size(), -1);
	int activeVerts = 0;
	
	// First pass: identify vertices used by any active triangle
	for (int i = 0; i < (int)_plist.size(); ++i) {
		if (_plist[i].isActive()) {
			int v[3];
			_plist[i].getVerts(v[0], v[1], v[2]);
			for (int k = 0; k < 3; k++) {
				if (v[k] >= 0 && v[k] < (int)_vlist.size()) {
					if (newId[v[k]] == -1) {
						newId[v[k]] = 0; // mark as found
					}
				}
			}
		}
	}

	// Assign new IDs only to those vertices that are used by active triangles
	for (int i = 0; i < (int)_vlist.size(); ++i) {
		if (newId[i] != -1) {
			newId[i] = activeVerts++;
		}
	}

	// Calculate the number of triangles that will be written (those with 3 valid vertices)
	int writtenTris = 0;
	for (int i = 0; i < (int)_plist.size(); ++i) {
		if (_plist[i].isActive()) {
			int v[3];
			_plist[i].getVerts(v[0], v[1], v[2]);
			if (v[0] >= 0 && v[1] >= 0 && v[2] >= 0 && 
				newId[v[0]] != -1 && newId[v[1]] != -1 && newId[v[2]] != -1) {
				writtenTris++;
			}
		}
	}

	fprintf(fp, "ply\n");
	fprintf(fp, "format ascii 1.0\n");
	fprintf(fp, "element vertex %d\n", activeVerts);
	fprintf(fp, "property float x\n");
	fprintf(fp, "property float y\n");
	fprintf(fp, "property float z\n");
	fprintf(fp, "element face %d\n", writtenTris);
	fprintf(fp, "property list uchar int vertex_indices\n");
	fprintf(fp, "end_header\n");

	// Write vertices
	for (int i = 0; i < (int)_vlist.size(); ++i) {
		if (newId[i] != -1) {
			const Vec3& v = _vlist[i].getXYZ();
			fprintf(fp, "%f %f %f\n", v.x, v.y, v.z);
		}
	}

	// Write faces
	for (int i = 0; i < (int)_plist.size(); ++i) {
		if (_plist[i].isActive()) {
			int v[3];
			_plist[i].getVerts(v[0], v[1], v[2]);
			if (v[0] >= 0 && v[1] >= 0 && v[2] >= 0 && 
				newId[v[0]] != -1 && newId[v[1]] != -1 && newId[v[2]] != -1) {
				fprintf(fp, "3 %d %d %d\n", newId[v[0]], newId[v[1]], newId[v[2]]);
			}
		}
	}

	fclose(fp);
	return true;
}


bool Mesh::saveToSTL(char* filename)
{
	if (_isStlBinary) {
		FILE* fp = fopen(filename, "wb");
		if (!fp) return false;

		char header[80] = {0};
		const char* hstr = "simplified_mesh_exported_by_mesh_processor";
		for(int i = 0; i < 80 && hstr[i] != '\0'; i++) header[i] = hstr[i];
		fwrite(header, 1, 80, fp);

		// Count active triangles
		unsigned int activeTris = 0;
		for (int i = 0; i < (int)_plist.size(); ++i) {
			if (_plist[i].isActive()) activeTris++;
		}
		fwrite(&activeTris, 4, 1, fp);

		for (int i = 0; i < (int)_plist.size(); ++i) {
			if (_plist[i].isActive()) {
				float n[3];
				const Vec3& normal = _plist[i].getNormalVec3();
				n[0] = normal.x; n[1] = normal.y; n[2] = normal.z;
				fwrite((void*)n, 4, 3, fp);

				int v1[3];
				_plist[i].getVerts(v1[0], v1[1], v1[2]);
				for (int k = 0; k < 3; k++) {
					float v[3];
					const Vec3& p = _vlist[v1[k]].getXYZ();
					v[0] = p.x; v[1] = p.y; v[2] = p.z;
					fwrite((void*)v, 4, 3, fp);
				}

				unsigned short attr = 0;
				fwrite(&attr, 2, 1, fp);
			}
		}

		fclose(fp);
		return true;

	} else {
		FILE* fp = fopen(filename, "w");
		if (!fp) return false;

		fprintf(fp, "solid simplified_mesh\n");

		for (int i = 0; i < (int)_plist.size(); ++i) {
			if (_plist[i].isActive()) {
				const Vec3& n = _plist[i].getNormalVec3();
				int v1[3];
				_plist[i].getVerts(v1[0], v1[1], v1[2]);
				
				fprintf(fp, "  facet normal %f %f %f\n", n.x, n.y, n.z);
				fprintf(fp, "    outer loop\n");
				for (int k = 0; k < 3; k++) {
					const Vec3& p = _vlist[v1[k]].getXYZ();
					fprintf(fp, "      vertex %f %f %f\n", p.x, p.y, p.z);
				}
				fprintf(fp, "    endloop\n");
				fprintf(fp, "  endfacet\n");
			}
		}

		fprintf(fp, "endsolid simplified_mesh\n");
		fclose(fp);
		return true;
	}
}

bool Mesh::saveToOBJ(char* filename)
{
	FILE* fp = fopen(filename, "w");
	if (!fp) return false;

	vector<int> newId(_vlist.size(), -1);
	int activeVerts = 0;

	for (int i = 0; i < (int)_plist.size(); ++i) {
		if (_plist[i].isActive()) {
			int v[3];
			_plist[i].getVerts(v[0], v[1], v[2]);
			for (int k = 0; k < 3; k++) {
				if (v[k] >= 0 && v[k] < (int)_vlist.size()) {
					if (newId[v[k]] == -1) {
						newId[v[k]] = ++activeVerts;
					}
				}
			}
		}
	}

	fprintf(fp, "# OBJ file saved by Mesh Processor\n");
	for (int i = 0; i < (int)_vlist.size(); ++i) {
		if (newId[i] != -1) {
			const Vec3& v = _vlist[i].getXYZ();
			fprintf(fp, "v %f %f %f\n", v.x, v.y, v.z);
		}
	}

	for (int i = 0; i < (int)_plist.size(); ++i) {
		if (_plist[i].isActive()) {
			int v[3];
			_plist[i].getVerts(v[0], v[1], v[2]);
			if (v[0] >= 0 && v[1] >= 0 && v[2] >= 0 && 
				newId[v[0]] != -1 && newId[v[1]] != -1 && newId[v[2]] != -1) {
				fprintf(fp, "f %d %d %d\n", newId[v[0]], newId[v[1]], newId[v[2]]);
			}
		}
	}

	fclose(fp);
	return true;
}

// Used for debugging
void Mesh::dump()
{
	std::cout << "*** Mesh Dump ***" << std::endl;
	std::cout << "# of vertices: " << _numVerts << std::endl;
	std::cout << "# of triangles: " << _numTriangles << std::endl;
	for (unsigned i = 0; i < _vlist.size(); ++i)
	{
		std::cout << "\tVertex " << i << ": " << _vlist[i] << std::endl;
	}
	std::cout << std::endl;
	for (unsigned i = 0; i < _plist.size(); ++i)
	{
		std::cout << "\tTriangle " << i << ": " << _plist[i] << std::endl;
	}
	std::cout << "*** End of Mesh Dump ***" << std::endl;
	std::cout << std::endl;
}


