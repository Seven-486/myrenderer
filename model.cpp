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
        std::istringstream iss(line);
        char trash;
        if(!line.compare(0,2,"v ")){
            iss>>trash;
            vec<3> v;
            for(int i=0;i<3;i++) iss>>v[i];
            verts_.push_back(v);
        }else if(!line.compare(0,2,"f ")){
            std::vector<int> f;
            int itrash,n;
            iss>>trash;
            while(iss>>n>>trash>>itrash>>trash>>itrash){
                n--;
                f.push_back(n);
            }
            faces_.push_back(f);
        }
    }
    std::cerr<< "# v# "<<verts_.size()<<" vn# "<<faces_.size()<<std::endl;
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
