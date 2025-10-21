#include<vector>
#include<geometry.hpp>
class model{
    private:
    std::vector<vec<3>> verts_;
    std::vector<std::vector<int>> faces_;

    public:
    model(const char* filename);
    int nverts() const;
    int nfaces()const;
    vec3 vert(const int i)const ;
    vec3 vert(const int iface, const int nthvert) const; 
};