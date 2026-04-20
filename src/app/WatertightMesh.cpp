// ============================================================
//  WatertightMesh.cpp — Mayo wrapper for mesh_repair logic
//
//  IMPORTANT: mesh_repair.cpp uses:
//    - #include <windows.h>  (Windows file dialog)
//    - using namespace std;
//    - cout for progress
//
//  We do NOT include mesh_repair.cpp directly.
//  Instead we copy only the functions we need here,
//  cleaned up for Mayo (no windows.h, no cout, no dialogs).
//  All logic is identical to mesh_repair.cpp.
// ============================================================

#include "WatertightMesh.h"

#include <algorithm>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>
#include <cmath>

namespace {

// ── Data structures (same as mesh_repair.h but in anon namespace) ────────────

struct Vertex { float x, y, z; };
struct Face   { std::vector<int> vertexIndices; };
struct Mesh   { std::vector<Vertex> vertices; std::vector<Face> faces; };

struct Vec3 {
    float x=0,y=0,z=0;
    Vec3 operator-(const Vec3& v)const{return{x-v.x,y-v.y,z-v.z};}
    Vec3 operator+(const Vec3& v)const{return{x+v.x,y+v.y,z+v.z};}
    Vec3 operator*(float s)      const{return{x*s,  y*s,  z*s  };}
    bool operator<(const Vec3& o)const{
        if(x!=o.x)return x<o.x;
        if(y!=o.y)return y<o.y;
        return z<o.z;
    }
};

struct BBox { Vec3 min, max; };

// ── Loaders (same logic as mesh_repair.cpp, no Windows deps) ─────────────────

bool loadOBJ(const std::string& filename, Mesh& mesh){
    std::ifstream file(filename);
    if(!file.is_open())return false;
    std::string line;
    mesh.vertices.clear();mesh.faces.clear();
    while(std::getline(file,line)){
        std::stringstream ss(line);std::string type;ss>>type;
        if(type=="v"){
            Vertex v;ss>>v.x>>v.y>>v.z;mesh.vertices.push_back(v);
        }else if(type=="f"){
            Face f;std::string segment;
            while(ss>>segment){
                size_t firstSlash=segment.find('/');
                try{int index=std::stoi(segment.substr(0,firstSlash))-1;f.vertexIndices.push_back(index);}
                catch(...){}
            }
            if(!f.vertexIndices.empty())mesh.faces.push_back(f);
        }
    }
    return true;
}

bool loadSTL(const std::string& filename, Mesh& mesh){
    std::ifstream file(filename,std::ios::binary);
    if(!file.is_open())return false;
    char header[5];file.read(header,5);
    bool isASCII=(std::string(header,5)=="solid");
    file.seekg(0);
    mesh.vertices.clear();mesh.faces.clear();
    if(isASCII){
        std::string line;
        while(std::getline(file,line)){
            std::stringstream ss(line);std::string word;ss>>word;
            if(word=="vertex"){
                Vertex v;ss>>v.x>>v.y>>v.z;mesh.vertices.push_back(v);
                if(mesh.vertices.size()%3==0){
                    Face f;int last=(int)mesh.vertices.size()-1;
                    f.vertexIndices={last-2,last-1,last};mesh.faces.push_back(f);
                }
            }
        }
    }else{
        file.seekg(80);uint32_t numFaces;file.read((char*)&numFaces,4);
        for(uint32_t i=0;i<numFaces;++i){
            float n[3];file.read((char*)n,12);
            Vertex v[3];
            for(int k=0;k<3;++k){
                file.read((char*)&v[k].x,4);file.read((char*)&v[k].y,4);file.read((char*)&v[k].z,4);
                mesh.vertices.push_back(v[k]);
            }
            uint16_t attr;file.read((char*)&attr,2);
            Face f;int last=(int)mesh.vertices.size()-1;
            f.vertexIndices={last-2,last-1,last};mesh.faces.push_back(f);
        }
    }
    return true;
}

bool loadPLY(const std::string& filename, Mesh& mesh){
    std::ifstream file(filename);
    if(!file.is_open())return false;
    std::string line;int numVertices=0,numFaces=0;
    while(std::getline(file,line)){
        if(line.find("element vertex")!=std::string::npos)numVertices=std::stoi(line.substr(15));
        if(line.find("element face")!=std::string::npos)numFaces=std::stoi(line.substr(13));
        if(line=="end_header")break;
    }
    mesh.vertices.clear();mesh.faces.clear();
    for(int i=0;i<numVertices;++i){Vertex v;file>>v.x>>v.y>>v.z;mesh.vertices.push_back(v);}
    for(int i=0;i<numFaces;++i){
        int count;file>>count;Face f;
        for(int j=0;j<count;++j){int idx;file>>idx;f.vertexIndices.push_back(idx);}
        mesh.faces.push_back(f);
    }
    return true;
}

bool loadMesh(const std::string& filename, Mesh& mesh){
    std::string ext=filename.substr(filename.find_last_of(".")+1);
    std::transform(ext.begin(),ext.end(),ext.begin(),::tolower);
    if(ext=="obj")return loadOBJ(filename,mesh);
    if(ext=="stl")return loadSTL(filename,mesh);
    if(ext=="ply")return loadPLY(filename,mesh);
    return false;
}

void saveOBJ(const std::string& filename, const Mesh& mesh){
    std::ofstream file(filename);
    for(const auto& v:mesh.vertices)
        file<<"v "<<v.x<<" "<<v.y<<" "<<v.z<<"\n";
    for(const auto& f:mesh.faces){
        file<<"f";
        for(int idx:f.vertexIndices)file<<" "<<(idx+1);
        file<<"\n";
    }
}

// ── Hole filling (same logic as mesh_repair.cpp) ──────────────────────────────

void fillHolesSurfaceStyle(Mesh& mesh){
    // Build edge → face map
    std::map<std::pair<int,int>,std::vector<int>> edgeMap;
    for(int fi=0;fi<(int)mesh.faces.size();++fi){
        const auto& f=mesh.faces[fi];
        int n=(int)f.vertexIndices.size();
        for(int i=0;i<n;++i){
            int a=f.vertexIndices[i],b=f.vertexIndices[(i+1)%n];
            if(a>b)std::swap(a,b);
            edgeMap[{a,b}].push_back(fi);
        }
    }
    // Collect boundary edges (used by exactly 1 face)
    std::map<int,std::vector<int>> boundaryAdj;
    for(const auto& [e,faces]:edgeMap){
        if(faces.size()==1){
            boundaryAdj[e.first].push_back(e.second);
            boundaryAdj[e.second].push_back(e.first);
        }
    }
    std::set<int> visited;
    int holesFilled=0,newFacesCount=0;
    for(auto& [startV,_]:boundaryAdj){
        if(visited.count(startV))continue;
        // Walk boundary loop
        std::vector<int> loop;
        int cur=startV,prev=-1;
        while(true){
            visited.insert(cur);loop.push_back(cur);
            int next=-1;
            for(int nb:boundaryAdj[cur]){
                if(nb!=prev&&!visited.count(nb)){next=nb;break;}
            }
            if(next==-1)break;
            prev=cur;cur=next;
        }
        if(loop.size()<3)continue;
        // Add centroid vertex
        float cx=0,cy=0,cz=0;
        for(int vi:loop){cx+=mesh.vertices[vi].x;cy+=mesh.vertices[vi].y;cz+=mesh.vertices[vi].z;}
        int n=(int)loop.size();
        mesh.vertices.push_back({cx/n,cy/n,cz/n});
        int centroidIdx=(int)mesh.vertices.size()-1;
        for(int i=0;i<n;++i){
            Face f;f.vertexIndices={loop[i],loop[(i+1)%n],centroidIdx};
            mesh.faces.push_back(f);newFacesCount++;
        }
        holesFilled++;
    }
}

// ── Voxel grid (same as mesh_repair.cpp) ─────────────────────────────────────

struct VoxelGrid {
    int res;
    std::vector<uint8_t> data;
    Vec3 minBound,maxBound,scale;

    VoxelGrid(int r,BBox box):res(r){
        data.resize(res*res*res,0);
        Vec3 span=box.max-box.min;
        minBound=box.min-span*0.05f;
        maxBound=box.max+span*0.05f;
        Vec3 s=maxBound-minBound;
        scale={(float)res/s.x,(float)res/s.y,(float)res/s.z};
    }
    void set(int x,int y,int z,uint8_t val){
        if(x>=0&&x<res&&y>=0&&y<res&&z>=0&&z<res)
            data[x*res*res+y*res+z]=val;
    }
    uint8_t get(int x,int y,int z)const{
        if(x>=0&&x<res&&y>=0&&y<res&&z>=0&&z<res)
            return data[x*res*res+y*res+z];
        return 2;
    }
    Vec3 gridToWorld(int x,int y,int z)const{
        return{minBound.x+x/scale.x,minBound.y+y/scale.y,minBound.z+z/scale.z};
    }
};

void rasterizeMesh(const Mesh& mesh,VoxelGrid& grid){
    for(const auto& f:mesh.faces){
        if(f.vertexIndices.size()<3)continue;
        Vec3 v0={mesh.vertices[f.vertexIndices[0]].x,mesh.vertices[f.vertexIndices[0]].y,mesh.vertices[f.vertexIndices[0]].z};
        Vec3 v1={mesh.vertices[f.vertexIndices[1]].x,mesh.vertices[f.vertexIndices[1]].y,mesh.vertices[f.vertexIndices[1]].z};
        Vec3 v2={mesh.vertices[f.vertexIndices[2]].x,mesh.vertices[f.vertexIndices[2]].y,mesh.vertices[f.vertexIndices[2]].z};
        float edgeMax=0;
        auto upd=[&](float v){if(std::abs(v)>edgeMax)edgeMax=std::abs(v);};
        upd(v1.x-v0.x);upd(v1.y-v0.y);upd(v1.z-v0.z);
        upd(v2.x-v0.x);upd(v2.y-v0.y);upd(v2.z-v0.z);
        float maxScale=std::max({grid.scale.x,grid.scale.y,grid.scale.z});
        int steps=std::min((int)(edgeMax*maxScale)+2,100);
        for(int i=0;i<=steps;++i){
            for(int j=0;j<=steps-i;++j){
                float u=(float)i/steps,v=(float)j/steps;
                Vec3 p=v0*(1.f-u-v)+v1*u+v2*v;
                int gx=(int)((p.x-grid.minBound.x)*grid.scale.x);
                int gy=(int)((p.y-grid.minBound.y)*grid.scale.y);
                int gz=(int)((p.z-grid.minBound.z)*grid.scale.z);
                for(int dx=0;dx<=1;++dx)
                    for(int dy=0;dy<=1;++dy)
                        for(int dz=0;dz<=1;++dz)
                            grid.set(gx+dx,gy+dy,gz+dz,1);
            }
        }
    }
}

void floodFillOutside(VoxelGrid& grid){
    struct Node{int x,y,z;};
    std::vector<Node> q;
    q.push_back({0,0,0});grid.set(0,0,0,2);
    int dx[]={1,-1,0,0,0,0},dy[]={0,0,1,-1,0,0},dz[]={0,0,0,0,1,-1};
    size_t head=0;
    while(head<q.size()){
        Node curr=q[head++];
        for(int i=0;i<6;++i){
            int nx=curr.x+dx[i],ny=curr.y+dy[i],nz=curr.z+dz[i];
            if(nx>=0&&nx<grid.res&&ny>=0&&ny<grid.res&&nz>=0&&nz<grid.res){
                if(grid.get(nx,ny,nz)==0){grid.set(nx,ny,nz,2);q.push_back({nx,ny,nz});}
            }
        }
    }
}

void extractVoxelSurface(VoxelGrid& grid,Mesh& outMesh){
    outMesh.vertices.clear();outMesh.faces.clear();
    std::map<std::tuple<int,int,int>,int> vertMap;
    auto getV=[&](int x,int y,int z){
        auto key=std::make_tuple(x,y,z);
        if(vertMap.count(key))return vertMap[key];
        Vec3 p=grid.gridToWorld(x,y,z);
        outMesh.vertices.push_back({p.x,p.y,p.z});
        return vertMap[key]=(int)outMesh.vertices.size()-1;
    };
    int dx[]={1,-1,0,0,0,0},dy[]={0,0,1,-1,0,0},dz[]={0,0,0,0,1,-1};
    for(int x=0;x<grid.res;++x)for(int y=0;y<grid.res;++y)for(int z=0;z<grid.res;++z){
        if(grid.get(x,y,z)!=2){
            for(int i=0;i<6;++i){
                if(grid.get(x+dx[i],y+dy[i],z+dz[i])==2){
                    Face f;
                    if(i==0)f.vertexIndices={getV(x+1,y,z),getV(x+1,y+1,z),getV(x+1,y+1,z+1),getV(x+1,y,z+1)};
                    else if(i==1)f.vertexIndices={getV(x,y,z),getV(x,y,z+1),getV(x,y+1,z+1),getV(x,y+1,z)};
                    else if(i==2)f.vertexIndices={getV(x,y+1,z),getV(x,y+1,z+1),getV(x+1,y+1,z+1),getV(x+1,y+1,z)};
                    else if(i==3)f.vertexIndices={getV(x,y,z),getV(x+1,y,z),getV(x+1,y,z+1),getV(x,y,z+1)};
                    else if(i==4)f.vertexIndices={getV(x,y,z+1),getV(x+1,y,z+1),getV(x+1,y+1,z+1),getV(x,y+1,z+1)};
                    else f.vertexIndices={getV(x,y,z),getV(x,y+1,z),getV(x+1,y+1,z),getV(x+1,y,z)};
                    outMesh.faces.push_back(f);
                }
            }
        }
    }
}

void smoothMesh(Mesh& mesh,int iterations){
    for(int iter=0;iter<iterations;++iter){
        std::vector<Vertex> newVerts=mesh.vertices;
        std::vector<int> counts(mesh.vertices.size(),0);
        std::vector<Vec3> sums(mesh.vertices.size(),{0,0,0});
        for(const auto& f:mesh.faces){
            for(size_t i=0;i<f.vertexIndices.size();++i){
                int v1=f.vertexIndices[i];
                int v2=f.vertexIndices[(i+1)%f.vertexIndices.size()];
                sums[v1]=sums[v1]+Vec3{mesh.vertices[v2].x,mesh.vertices[v2].y,mesh.vertices[v2].z};
                counts[v1]++;
                sums[v2]=sums[v2]+Vec3{mesh.vertices[v1].x,mesh.vertices[v1].y,mesh.vertices[v1].z};
                counts[v2]++;
            }
        }
        for(size_t i=0;i<mesh.vertices.size();++i){
            if(counts[i]>0){
                Vec3 avg=sums[i]*(1.f/counts[i]);
                newVerts[i]={avg.x,avg.y,avg.z};
            }
        }
        mesh.vertices=newVerts;
    }
}

} // anonymous namespace

// ── Public Mayo API ───────────────────────────────────────────────────────────

namespace Mayo {

std::string wtRepairMesh(
    const std::string& inputPath,
    const std::string& outputPath,
    int  voxelResolution,
    WTProgressCallback cb)
{
    auto report=[&](int pct,const std::string& msg){if(cb)cb(pct,msg);};

    report(0,"Loading mesh…");
    Mesh mesh;
    if(!loadMesh(inputPath,mesh))
        return "Failed to load: "+inputPath;
    if(mesh.faces.empty())
        return "Mesh has no faces.";

    report(10,"Stage 1: Filling surface holes…");
    fillHolesSurfaceStyle(mesh);

    report(25,"Computing bounding box…");
    BBox bbox;
    bbox.min=bbox.max={mesh.vertices[0].x,mesh.vertices[0].y,mesh.vertices[0].z};
    for(const auto& v:mesh.vertices){
        bbox.min.x=std::min(bbox.min.x,v.x);bbox.min.y=std::min(bbox.min.y,v.y);bbox.min.z=std::min(bbox.min.z,v.z);
        bbox.max.x=std::max(bbox.max.x,v.x);bbox.max.y=std::max(bbox.max.y,v.y);bbox.max.z=std::max(bbox.max.z,v.z);
    }

    report(30,"Stage 2: Building voxel grid (res="+std::to_string(voxelResolution)+")…");
    VoxelGrid grid(voxelResolution,bbox);

    report(35,"Rasterizing mesh into voxels…");
    rasterizeMesh(mesh,grid);

    report(60,"Flood-filling exterior…");
    floodFillOutside(grid);

    report(75,"Extracting watertight surface…");
    Mesh repaired;
    extractVoxelSurface(grid,repaired);

    report(88,"Smoothing (3 passes)…");
    smoothMesh(repaired,3);

    report(95,"Saving OBJ…");
    saveOBJ(outputPath,repaired);

    report(100,"Done. Vertices: "+std::to_string(repaired.vertices.size())+
        "  Faces: "+std::to_string(repaired.faces.size()));
    return ""; // success
}

} // namespace Mayo
