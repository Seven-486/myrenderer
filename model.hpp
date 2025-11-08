#include<vector>
#include<geometry.hpp>
#include<tgaimage.hpp>
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
    TGAImage texture_normal,
    texture_color,
    texture_spec;
    

    public:
    model(const char* filename);
    const TGAImage& get_texture_color() const;
    const TGAImage& get_texture_normal() const;
    const TGAImage& get_texture_spec() const;
    void read_texture(const char* filename);
    void read_specmap(const char* filename);
    void read_normalmap(const char* filename);
    int nverts() const;          //获取顶点数量
    int nfaces()const;           //获取面片数量
    vec3 vert(const int i)const ;          //获取第i个顶点的坐标
    vec3 vert(const int iface, const int nthvert) const;  //获取第iface个面片的第nthvert个顶点的坐标
    vec3 normal(const int iface, const int nthvert) const; //获取第iface个面片的第nthvert个顶点的法线坐标
    vec4 normal(const vec2 &uv) const;                      //通过纹理坐标获取法线贴图的法线
    vec2 tex(const int iface, const int nthvert) const;

};