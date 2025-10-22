#include "model.hpp"
#include<fstream>
#include<sstream>
#include<string>
#include<iostream>
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
            for(int i=0;i<3;i++) iss>>v[i]; 
            verts_.push_back(v);
        }else if(!line.compare(0,2,"f ")){ // line starts with "f "
            std::vector<int> f; 
            int itrash,n; //读取第一个，第四个，第七个数组成一个顶点的索引，忽略其他的
            iss>>trash;//
            while(iss>>n>>trash>>itrash>>trash>>itrash){ //trash是一个临时变量，用来读取斜杠,itrash是用来读取被忽略的值,空格分隔则是自动跳过
                n--; //  assume the obj file starts counting from 1
                f.push_back(n);
            }
            faces_.push_back(f);
        }
    }
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
