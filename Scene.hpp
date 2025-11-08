#pragma once
#include "model.hpp"
#include<vector>

class Scene{
    public:
    int weight=800;
    int height=800;
    Scene(int w,int h):weight(w),height(h){}
    TGAColor background_color={0,0,0,0};
    std::vector<model> models;
    void add_model(const model& m) {
        models.push_back(m);
    }
};