#include <cstdlib>
#include "our_gl.hpp"
#include "model.hpp"

inline mat<4,4> ModelView, Perspective,Modeltransform; // "OpenGL" state matrices and
inline std::vector<double> zbuffer;     // the depth buffer

struct RandomShader : IShader {
    const model &Model;
    TGAColor color = {};
    vec3 tri[3];  // triangle in eye coordinates

    RandomShader(const model &m) : Model(m) {
    }

    virtual vec4 vertex(const int face, const int vert) {
        vec3 v = Model.vert(face, vert);                          // current vertex in object coordinates
        vec4 gl_Position = ModelView * vec4{v.x, v.y, v.z, 1.};
        tri[vert] = gl_Position.xyz();                            // in eye coordinates
        return Perspective * gl_Position;                         // in clip coordinates
    }

    virtual std::pair<bool,TGAColor> fragment(const vec3 bar) const {
        return {false, color};                                    // do not discard the pixel
    }
};
int main(){
    constexpr int width  = 800;      // output image size
    constexpr int height = 800;
    constexpr vec3    eye{-1, 0, 2}; // camera position
    constexpr vec3 center{ 0, 0, 0}; // camera direction
    constexpr vec3     up{ 0, 1, 0}; // camera up vector

    lookat(eye, center, up);                                   // build the ModelView   matrix
    init_perspective_simple(norm(eye-center));                        // build the Perspective matrix
    init_viewport(width/16, height/16, width*7/8, height*7/8); // build the Viewport    matrix
    init_zbuffer(width, height);
    TGAImage framebuffer(width, height, TGAImage::RGB);
    model Model("../obj/african_head/african_head.obj"); RandomShader shader(Model);
    for(int i=0;i<Model.nfaces();i++){
        Triangle clip;
        shader.color = {static_cast<uint8_t>(std::rand()%256), static_cast<uint8_t>(std::rand()%256), static_cast<uint8_t>(std::rand()%256), 255};
        for(int j=0;j<3;j++){
            clip[j] = shader.vertex(i,j);
        }
        rasterize(clip,shader,framebuffer);
    }
    framebuffer.write_tga_file("output.tga");
    return 0;
}