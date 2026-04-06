#define _CRT_SECURE_NO_WARNINGS
#include <assert.h>
// ******************************
// mesh.h
//
// Mesh class, which stores a list
// of vertices & a list of triangles.

#ifndef __mesh_h
#define __mesh_h

#if defined (_MSC_VER) && (_MSC_VER >= 1020)

#pragma once
#pragma warning(disable:4710) // function not inlined
#pragma warning(disable:4702) // unreachable code
#pragma warning(disable:4514) // unreferenced inline function has been removed
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#endif


#include<fstream>
#include<string>
#include<iostream>
#include<math.h>

using namespace std;

#include <vector>
#include "vertex.h"
#include "triangle.h"

// Mesh class.  This stores a list of vertices &
// another list of triangles (which references the vertex list)
class Mesh
{
public:
	// Constructors and Destructors
	Mesh() { _numVerts = _numTriangles = 0; _isStlBinary = false; };
	Mesh(char* filename); // passed name of mesh file
	~Mesh();

	Mesh(const Mesh&); // copy ctor
	Mesh& operator=(const Mesh&); // assignment op

	// Get list of vertices, triangles
	vertex& getVertex(int index) { return _vlist[index]; };
	const vertex& getVertex(int index) const { return _vlist[index]; };
	triangle& getTri(int index) { return _plist[index]; };
	const triangle& getTri(int index) const { return _plist[index]; };

	// Add this overload to Mesh to match the usage in PMesh::getTri(int, triangle&)
	bool getTri(int index, triangle& t) const {
		if (index < 0 || index >= _numTriangles) return false;
		t = _plist[index];
		return true;
	}

	int getNumVerts() { return _numVerts; };
	void setNumVerts(int n) { _numVerts = n; };
	int getNumTriangles() { return _numTriangles; };
	void setNumTriangles(int n) { _numTriangles = n; };

	void Normalize();// center mesh around the origin & shrink to fit in [-1, 1]

	void calcOneVertNormal(unsigned vert); // recalc normal for one vertex

	void dump(); // print mesh state to cout

	bool saveToFile(char* filename); 
	bool saveToPLY(char* filename);
	bool saveToSTL(char* filename);
	bool saveToOBJ(char* filename);

private:
	vector<vertex> _vlist; // list of vertices in mesh
	vector<triangle> _plist; // list of triangles in mesh

	int _numVerts;
	int _numTriangles;

	enum PlyFormat { PLY_ASCII, PLY_BINARY_LE, PLY_BINARY_BE };
	PlyFormat _plyFormat;
	int _vertStride; 
	bool _hasTexCoords;
	int _xOff, _yOff, _zOff, _sOff, _tOff; // Offsets into binary vertex buffer
	int _xIdx, _yIdx, _zIdx, _sIdx, _tIdx;
	
	bool _isStlBinary; // Tracks whether loaded STL was binary

	bool loadFromFile(char* filename); // load from PLY or STL file
	bool loadFromSTLFile(char* filename); // load from STL file
	bool loadFromOBJFile(char* filename); // load from OBJ file

	void ChangeStrToLower(char* pszUpper)
	{
		size_t len = strlen(pszUpper);
		for (size_t i = 0; i < len; i++) {
			pszUpper[i] = (char)tolower(pszUpper[i]);
		}
	}

	// get bounding box for mesh
	void setMinMax(float min[3], float max[3]);

	void calcAllQMatrices(Mesh& mesh); // used for Quadrics method

	void calcVertNormals(); // Calculate the vertex normals after loading the mesh

	// Header parsing
	bool readPlyHeader(FILE*& inFile);
	bool readPlyVerts(FILE*& inFile);
	bool readPlyTris(FILE*& inFile);
};

#endif // __mesh_h
