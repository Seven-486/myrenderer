#include<vector>
#include<geometry.hpp>
struct FaceVertex{
    int vert_idx;
    int tex_idx;
    int norm_idx;
};
class model{
    private:
    std::vector<vec<3>> verts_;
    std::vector<std::vector<FaceVertex>> faces_;
    std::vector<vec<3>> normals_;

    public:
    model(const char* filename);
    int nverts() const;          //获取顶点数量
    int nfaces()const;           //获取面片数量
    vec3 vert(const int i)const ;          //获取第i个顶点的坐标
    vec3 vert(const int iface, const int nthvert) const;  //获取第iface个面片的第nthvert个顶点的坐标
    vec3 normal(const int iface, const int nthvert) const; //获取第iface个面片的第nthvert个顶点的法线坐标
};