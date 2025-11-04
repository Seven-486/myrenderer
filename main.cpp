#include <cstdlib>
#include "our_gl.hpp"
#include "model.hpp"

extern mat<4,4> ModelView, Perspective,Modeltransform; // "OpenGL" state matrices and
extern std::vector<double> zbuffer;     // the depth buffer
constexpr int width  = 800;      // output image size
constexpr int height = 800;
constexpr vec3    eye{-1, 0, 2}; // camera position
constexpr vec3 center{ 0, 0, 0}; // camera direction
constexpr vec3     up{ 0, 1, 0}; // camera up vector
constexpr vec4 light{ 1, 1, 1, 0}; // light direction
TGAImage texture;

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
struct PhongShader : IShader{
    const model& Model;
    vec4 light_dir;
    vec3 tri[3];  // triangle in eye coordinates
    vec3 tri_normal[3];
    TGAColor color = {};
    vec2 texcoord[3];  // triangle texture coordinates
    PhongShader(const vec4& light, const model& m) : Model(m), light_dir(light) {
               light_dir = normalized(ModelView * vec4{light.x, light.y, light.z, 0.});
    }
    virtual vec4 vertex(const int face, const int vert){
        vec3 v =Model.vert(face, vert);                          // current vertex in object coordinates
        vec2 varying_uv = Model.tex(face, vert);
        texcoord[vert] = varying_uv;
        //tri_normal[vert] =Model.normal(face, vert);                       // normal in object coordinates
        //tri_normal[vert] = (ModelView.invert_transpose() * vec4{tri_normal[vert].x, tri_normal[vert].y, tri_normal[vert].z, 0.}).xyz();
        vec4 gl_Position = ModelView * vec4{v.x, v.y, v.z, 1.};
        tri[vert] = gl_Position.xyz();                            // in eye
        return Perspective * gl_Position;                         // in clip coordinates
    }
    virtual std::pair<bool ,TGAColor> fragment(const vec3 bar)const {
         TGAColor gl_FragColor = {255, 255, 255, 255};             // output color of the fragment
        //vec3 n = normalized(cross(tri[1]-tri[0], tri[2]-tri[0])); // triangle normal in eye coordinates
        //vec3 n = normalized(tri_normal[0]*bar.x + tri_normal[1]*bar.y + tri_normal[2]*bar.z);
        //需要进行插值得出像素的纹理坐标
        vec2 uv = texcoord[0]*bar.x + texcoord[1]*bar.y + texcoord[2]*bar.z;
        TGAColor tex_color = texture.get(uv.x * texture.width(), uv.y * texture.height());
        //把tex_color的三个通道归一化到-1到1之间作为法线
        vec4 n = {
            (static_cast<double>(tex_color[2]) / 255.0) * 2.0 - 1.0,
            (static_cast<double>(tex_color[1]) / 255.0) * 2.0 - 1.0,
            (static_cast<double>(tex_color[0]) / 255.0) * 2.0 - 1.0,
            0.0
        };
        n = normalized(ModelView.invert_transpose() * n);
        
        vec4 r = normalized(n * (n * light_dir)*2 - light_dir);   // reflected light direction
        double ambient = .3;                                      // ambient light intensity
        double diff = std::max(0., n * light_dir);                        // diffuse light intensity
        double spec = std::pow(std::max(r.z, 0.), 35);                      // specular light intensity
        for(int channel:{0,1,2}){
            gl_FragColor[channel] *= std::min(ambient + 0.4*diff + 0.9*spec, 1.0);
        }
        return {false, gl_FragColor};
    }
    
};

int main(){
    init_modeltransform(1.0, {0,0,0}, {0,0,0});// --- IGNORE ---
    lookat(eye, center, up);                                   // build the ModelView   matrix
    init_perspective_simple(norm(eye-center));                        // build the Perspective matrix
    init_viewport(width/16, height/16, width*7/8, height*7/8); // build the Viewport    matrix
    init_zbuffer(width, height);
    TGAImage framebuffer(width, height, TGAImage::RGB);
    texture.read_tga_file("/Users/nasu/Documents/cpp_project/myrenderer/obj/african_head/african_head_nm.tga");

    model Model("../obj/african_head/african_head.obj");
  
    PhongShader shader(light, Model);
   
    for(int i=0;i<Model.nfaces();i++){
        Triangle clip;
        for(int j=0;j<3;j++){
            clip[j] = shader.vertex(i,j);
        }
        rasterize(clip,shader,framebuffer);
    }
   
    framebuffer.write_tga_file("output.tga");
    // TGAImage zbuffer_img(width, height, TGAImage::GRAYSCALE);

    // //转换zbuffer为图像，归一化到0-255
    // double zmin = *std::min_element(zbuffer.begin(), zbuffer.end());
    // double zmax = *std::max_element(zbuffer.begin(), zbuffer.end());
    // double z_second_min;
    // std::vector<double> z_copy = zbuffer;
    // std::sort(z_copy.begin(), z_copy.end());
    // for(const double& z : z_copy){
    //         if(z > zmin){
    //             z_second_min = z;
    //             break;
    //         }
    //     }
    // for(int x=0;x<width;x++){
    //     for(int y=0;y<height;y++){
    //         double zvalue = zbuffer[x + y * width];
    //         if(zvalue == std::numeric_limits<double>::max()){
    //             zbuffer_img.set(x, y, {0});
    //         } else {
    //             //线性归一化
    //             double gray = (255 * (zvalue - z_second_min) / (zmax - z_second_min));
    //             zbuffer_img.set(x, y, {static_cast<unsigned char>(gray)});
    //         }
    //     }
    // }
    // zbuffer_img.write_tga_file("zbuffer.tga");
    
    return 0;
}