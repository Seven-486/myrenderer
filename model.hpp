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
    std::vector<vec<2>> texcoords_;

    public:
    model(const char* filename);
    //如何加载两个文件到一起，可以通过
    int nverts() const;          //获取顶点数量
    int nfaces()const;           //获取面片数量
    vec3 vert(const int i)const ;          //获取第i个顶点的坐标
    vec3 vert(const int iface, const int nthvert) const;  //获取第iface个面片的第nthvert个顶点的坐标
    vec3 normal(const int iface, const int nthvert) const; //获取第iface个面片的第nthvert个顶点的法线坐标
    vec4 normal(const vec2 &uv) const;                      //通过纹理坐标获取法线贴图的法线
    vec2 tex(const int iface, const int nthvert) const;
};