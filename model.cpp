#include "model.hpp"
#include<fstream>
#include<sstream>
#include<string>
#include<iostream>
#include<iomanip>
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
            for(int i=0;i<3;){  
                iss>>v[i]; 
                if(v[i]=='-') { // handle negative numbers
                    iss>>v[i];
                    v[i] = -v[i];
                }
                i++;
            }
            verts_.push_back(v);
            for(int i:{0,1,2}){
                std::cerr<<std::right<<std::setw(2)<<v[i]<<" ";
            }
            std::cerr<<std::endl;
        }else if(!line.compare(0,2,"f ")){ // line starts with "f "
            std::vector<FaceVertex> f; 
            int texture_index, normal_index,vertex_index; 
            iss>>trash;// skip the "f" character
            while(iss>>vertex_index>>trash>>texture_index>>trash>>normal_index){ //trash是一个临时变量，用来读取斜杠,itrash是用来读取被忽略的值,空格分隔则是自动跳过
                vertex_index--; //  assume the obj file starts counting from 1 
                texture_index--;
                normal_index--;
                f.push_back({vertex_index, texture_index, normal_index});
            };
            faces_.push_back(f);

        }else if (!line.compare(0,3,"vn ")){ // line starts with "vn "

            iss>>trash>>trash; // skip the "vn" characters
            vec<3> n;
            for(int i=0;i<3;i++){
                iss>>n[i];  
            }
            normals_.push_back(n);
        }
        else if (!line.compare(0,3,"vt ")){ // line starts with "vt "

            iss>>trash>>trash; // skip the "vt" characters
            vec<2> tc;
            for(int i=0;i<2;i++){
                iss>>tc[i];  
            }
            texcoords_.push_back({tc.x,1-tc.y});
        }
    }
    //NormalizeToCenteredCube(verts_, true); // 归一化顶点坐标，保持长宽高比例
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
    return verts_[faces_[iface][nthvert].vert_idx];
}

vec3 model::normal(const int iface, const int nthvert) const
{
    return normals_[faces_[iface][nthvert].norm_idx];
}

// vec4 model::normal(const vec2 &uv) const
// {
//     //TGAColor c = normalmap.get(uv[0]*normalmap.width(), uv[1]*normalmap.height());
//     //return vec4{(double)c[2],(double)c[1],(double)c[0],0}*2./255. - vec4{1,1,1,0};
// }

vec2 model::tex(const int iface, const int nthvert) const
{
   return  texcoords_[faces_[iface][nthvert].tex_idx];
}
