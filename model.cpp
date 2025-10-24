#include "model.hpp"
#include<fstream>
#include<sstream>
#include<string>
#include<iostream>
struct BoundingBox { double minX, minY, minZ, maxX, maxY, maxZ; };
// 计算包围盒
BoundingBox CalculateBoundingBox(const std::vector<vec3>& vertices) {
    if (vertices.empty()) return {0, 0, 0, 0, 0, 0};
    double minX = vertices[0].x, minY = vertices[0].y, minZ = vertices[0].z;
    double maxX = minX, maxY = minY, maxZ = minZ;
    for (const auto& v : vertices) {
        minX = std::min(minX, v.x);
        minY = std::min(minY, v.y);
        minZ = std::min(minZ, v.z);
        maxX = std::max(maxX, v.x);
        maxY = std::max(maxY, v.y);
        maxZ = std::max(maxZ, v.z);
    }
    return {minX, minY, minZ, maxX, maxY, maxZ};
}
// 归一化到 [-1, 1]³
void NormalizeToCenteredCube(std::vector<vec3>& vertices, bool keepAspectRatio = false) {
    BoundingBox bbox = CalculateBoundingBox(vertices);
    
    // 计算包围盒中心和尺寸
    double centerX = (bbox.minX + bbox.maxX) * 0.5;
    double centerY = (bbox.minY + bbox.maxY) * 0.5;
    double centerZ = (bbox.minZ + bbox.maxZ) * 0.5;

    double sizeX = bbox.maxX - bbox.minX;
    double sizeY = bbox.maxY - bbox.minY;
    double sizeZ = bbox.maxZ - bbox.minZ;
    
    if (keepAspectRatio) {
        // 保持长宽比（按最大尺寸的一半缩放）
        double maxSizeHalf = std::max({sizeX, sizeY, sizeZ}) * 0.5;
        if (maxSizeHalf == 0) maxSizeHalf = 1.0; // 避免除以零
        
        for (auto& v : vertices) {
            v.x = (v.x - centerX) / maxSizeHalf;
            v.y = (v.y - centerY) / maxSizeHalf;
            v.z = (v.z - centerZ) / maxSizeHalf;
        }
    } else {
        // 各轴独立缩放
        double halfSizeX = sizeX * 0.5;
        double halfSizeY = sizeY * 0.5;
        double halfSizeZ = sizeZ * 0.5;

        if (halfSizeX == 0) halfSizeX = 1.0;
        if (halfSizeY == 0) halfSizeY = 1.0;
        if (halfSizeZ == 0) halfSizeZ = 1.0;

        for (auto& v : vertices) {
            v.x = (v.x - centerX) / halfSizeX;
            v.y = (v.y - centerY) / halfSizeY;
            v.z = (v.z - centerZ) / halfSizeZ;
        }
    }
}
model::model(const char* filename){
    std::ifstream in;
    in.open(filename,std::ifstream::in);
    if(in.fail()){
        std::cerr<<"file not found"<<std::endl;
        return;
    }
    std::string line;
    while(!in.eof()){
        std::getline(in,line);
        std::istringstream iss(line.c_str());
        char trash;
        if(!line.compare(0,2,"v ")){ // line starts with "v "
            
            iss>>trash; // skip the "v" character
            vec<3> v;
            for(int i=0;i<3;i++){
                iss>>v[i];  
            }
            verts_.push_back(v);
        }else if(!line.compare(0,2,"f ")){ // line starts with "f "
            std::vector<int> f; 
            int itrash,n; 
            iss>>trash;// skip the "f" character
            while(iss>>n>>trash>>itrash>>trash>>itrash){ //trash是一个临时变量，用来读取斜杠,itrash是用来读取被忽略的值,空格分隔则是自动跳过
                n--; //  assume the obj file starts counting from 1 
                f.push_back(n);
            }
            faces_.push_back(f);
        }
    }
    NormalizeToCenteredCube(verts_, true); // 归一化顶点坐标，保持长宽高比例
    std::cerr<< "# v# " <<verts_.size() << " f# "<< faces_.size() << std::endl;
}

int model::nverts()const{
    return verts_.size();
}

int model::nfaces()const {
    return faces_.size();
}

vec<3> model::vert(int i)const {
    return verts_[i];
}

vec3 model::vert(const int iface, const int nthvert) const {
    return verts_[faces_[iface][nthvert]];
}
