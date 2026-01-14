#include"our_gl.hpp"
#include<algorithm>

inline mat<4,4> ModelView, Viewport, Perspective,Modeltransform; // "OpenGL" state matrices
inline std::vector<double> zbuffer;               // depth buffer
inline std::vector<double> shadowbuffer;               // depth buffer for shadow map from light source

void init_modeltransform(double scale_factor, vec3 translate_vector, vec3 rotate_vector){
    Modeltransform = {{{scale_factor,0,0,translate_vector.x}, //功能是缩放和平移
                       {0,scale_factor,0,translate_vector.y},
                       {0,0,scale_factor,translate_vector.z},
                       {0,0,0,1}}};
    //旋转矩阵
    double rx = rotate_vector.x * M_PI / 180.0;
    double ry = rotate_vector.y * M_PI / 180.0;
    double rz = rotate_vector.z * M_PI / 180.0;

    mat<4,4> Rx = {{{1,0,0,0}, {0,cos(rx),-sin(rx),0}, {0,sin(rx),cos(rx),0}, {0,0,0,1}}};
    mat<4,4> Ry = {{{cos(ry),0,sin(ry),0}, {0,1,0,0}, {-sin(ry),0,cos(ry),0}, {0,0,0,1}}};
    mat<4,4> Rz = {{{cos(rz),-sin(rz),0,0}, {sin(rz),cos(rz),0,0}, {0,0,1,0}, {0,0,0,1}}};

    Modeltransform = Rz * Ry * Rx * Modeltransform;
}

void lookat(const vec3 eye, const vec3 center, const vec3 up) {
    vec3 n = normalized(eye-center);
    vec3 l = normalized(cross(up,n));
    vec3 m = normalized(cross(n, l));
    ModelView = mat<4,4>{{{l.x,l.y,l.z,0}, {m.x,m.y,m.z,0}, {n.x,n.y,n.z,0}, {0,0,0,1}}} *
                mat<4,4>{{{1,0,0,-center.x}, {0,1,0,-center.y}, {0,0,1,-center.z}, {0,0,0,1}}}; //moderview矩阵是mvp变换的view部分
}

void init_perspective_simple(const double f) { //简单透视投影矩阵,原理是将z值线性映射到[-1,1]
    Perspective = {{{1,0,0,0}, {0,1,0,0}, {0,0,1,0}, {0,0, -1/f,1}}};
}

void init_viewport(const int x, const int y, const int w, const int h) {
    Viewport = {{{w/2., 0, 0, x+w/2.}, {0, h/2., 0, y+h/2.}, {0,0,1,0}, {0,0,0,1}}};    //viewport矩阵是mvp变换的最后一步，将ndc坐标映射到屏幕坐标
}

void init_zbuffer(const int width, const int height) {
    zbuffer.resize(width * height);
    std::fill(zbuffer.begin(), zbuffer.end(), -std::numeric_limits<double>::max());
}

void init_shadowbuffer(const int width, const int height)
{
    shadowbuffer.resize(width * height);
    std::fill(shadowbuffer.begin(), shadowbuffer.end(), -std::numeric_limits<double>::max());
}

void rasterize(const Triangle &clip, const IShader &shader, TGAImage &framebuffer) {
    vec4 ndc[3]    = { clip[0]/clip[0].w, clip[1]/clip[1].w, clip[2]/clip[2].w };                // normalized device coordinates
    vec2 screen[3] = { (Viewport*ndc[0]).xy(), (Viewport*ndc[1]).xy(), (Viewport*ndc[2]).xy() }; // screen coordinates

    mat<3,3> ABC = {{ {screen[0].x, screen[0].y, 1.}, {screen[1].x, screen[1].y, 1.}, {screen[2].x, screen[2].y, 1.} }};
    if (ABC.det()<1) return; // backface culling + discarding triangles that cover less than a pixel

    auto [bbminx,bbmaxx] = std::minmax({screen[0].x, screen[1].x, screen[2].x}); // bounding box for the triangle
    auto [bbminy,bbmaxy] = std::minmax({screen[0].y, screen[1].y, screen[2].y}); // defined by its top left and bottom right corners
#pragma omp parallel for
    for (int x=std::max<int>(bbminx, 0); x<=std::min<int>(bbmaxx, framebuffer.width()-1); x++) {         // clip the bounding box by the screen
        for (int y=std::max<int>(bbminy, 0); y<=std::min<int>(bbmaxy, framebuffer.height()-1); y++) {
            vec3 bc = ABC.invert_transpose() * vec3{static_cast<double>(x), static_cast<double>(y), 1.}; // barycentric coordinates of {x,y} w.r.t the triangle
            if (bc.x<0 || bc.y<0 || bc.z<0) continue;                                                    // negative barycentric coordinate => the pixel is outside the triangle
            double z = bc * vec3{ ndc[0].z, ndc[1].z, ndc[2].z };  // linear interpolation of the depth
            if (z <= zbuffer[x+y*framebuffer.width()]) continue;   // discard fragments that are too deep w.r.t the z-buffer
            auto [discard, color] = shader.fragment(bc);
            if (discard) continue;                                 // fragment shader can discard current fragment
            zbuffer[x+y*framebuffer.width()] = z;                  // update the z-buffer
            framebuffer.set(x, y, color);                          // update the framebuffer
        }
    }
}

void rasterize_msaa4x(const Triangle &clip, const IShader &shader, TGAImage &framebuffer) {
    
}