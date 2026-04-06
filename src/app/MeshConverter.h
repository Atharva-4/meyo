#pragma once
#define _CRT_SECURE_NO_WARNINGS

#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <string>
#include <cstring>
#include <algorithm>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <limits>

using namespace std;

// ─── Data Structures ───────────────────────────────────────────────────────

struct Vertex {
    float x, y, z;
};

struct Face {
    vector<int> indices;
};

struct Triangle {
    float nx, ny, nz;
    Vertex v1, v2, v3;
};

// ─── Global Mesh Data ──────────────────────────────────────────────────────

extern vector<Vertex> vertices;
extern vector<Face>   faces;

// ─── Utility ───────────────────────────────────────────────────────────────

bool   fileExists(const string& name);
string trim(const string& s);

// ─── Readers ───────────────────────────────────────────────────────────────

bool readPLY(string filename);
bool readSTL(string filename);
bool readOBJ(string filename);

// ─── Writers ───────────────────────────────────────────────────────────────

bool writePLY     (string filename);
bool writeSTLBinary(string filename);
bool writeSTLAscii (string filename);
bool writeOBJ     (string filename);
