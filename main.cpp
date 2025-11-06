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
TGAImage texture_normal,texture_color,texture_spec;

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

    double inv_w[3];          // 1/w 的倒数
    vec2 uv_persp[3];         // 透视矫正的 uv/w
    vec4 nrm_persp[3];        // 透视矫正的法线/w

    vec4 tri[3];  // triangle in eye coordinates
    vec4 varying_nrm[3];
    vec2 texcoord[3];  // triangle texture coordinates
    PhongShader(const vec4& light, const model& m) : Model(m), light_dir(light) {
               light_dir = normalized(ModelView * vec4{light.x, light.y, light.z, 0.});
    }
    virtual vec4 vertex(const int face, const int vert){
        vec3 v =Model.vert(face, vert);                          // current vertex in object coordinates
        texcoord[vert] = Model.tex(face, vert);
        varying_nrm[vert] =vec4{Model.normal(face, vert).x,Model.normal(face, vert).y,Model.normal(face, vert).z, 0.0};                       // normal in object coordinates
        varying_nrm[vert] = (ModelView.invert_transpose() * vec4{varying_nrm[vert].x, varying_nrm[vert].y, varying_nrm[vert].z, 0.});
        vec4 gl_Position = ModelView * vec4{v.x, v.y, v.z, 1.};
        tri[vert] = gl_Position; 
        gl_Position = Perspective * gl_Position;  
        inv_w[vert] = 1.0 / gl_Position.w;
        uv_persp[vert] = texcoord[vert] * inv_w[vert];
        nrm_persp[vert] = varying_nrm[vert] * inv_w[vert];
        return gl_Position;                         // in clip coordinates
    }
    virtual std::pair<bool ,TGAColor> fragment(const vec3 bar)const {
        // output color of the fragment
        //vec3 n = normalized(cross(tri[1]-tri[0], tri[2]-tri[0])); // triangle normal in eye coordinates
        //vec3 n = normalized(tri_normal[0]*bar.x + tri_normal[1]*bar.y + tri_normal[2]*bar.z);
       double inv_w_interp = inv_w[0] * bar.x + inv_w[1] * bar.y + inv_w[2] * bar.z;
       double w_interp = 1.0 / inv_w_interp;  // 当前片段的 w

       mat<2,4> E ={ tri[1]-tri[0], tri[2]-tri[0]};
       mat<2,2> U ={texcoord[1]-texcoord[0], texcoord[2]-texcoord[0]};
       mat<2,4> T = U.invert()*E; //切线空间到眼睛空间的变换矩阵
       vec4 interpolated_normal = normalized(
            (nrm_persp[0]*bar.x + nrm_persp[1]*bar.y + nrm_persp[2]*bar.z)*w_interp
        );
        mat<4,4> D = { normalized(T[0]), normalized(T[1]),interpolated_normal, {0,0,0,1} };

        vec2 uv = (uv_persp[0]*bar.x + uv_persp[1]*bar.y + uv_persp[2]*bar.z)*w_interp; 
        TGAColor diffuse_color= texture_color.get(uv.x * texture_color.width(), uv.y * texture_color.height());
        double spec_intensity = texture_spec.get(uv.x * texture_spec.width(), uv.y * texture_spec.height())[0] / 255.0;
        TGAColor tex_normal = texture_normal.get(uv.x * texture_normal.width(), uv.y * texture_normal.height());
        //把三个通道归一化到-1到1之间作为法线
        vec4 tangent_normal = {
            (static_cast<double>(tex_normal[2]) / 255.0) * 2.0 - 1.0,
            (static_cast<double>(tex_normal[1]) / 255.0) * 2.0 - 1.0,
            (static_cast<double>(tex_normal[0]) / 255.0) * 2.0 - 1.0,
            0.0
        };
        
        vec4 n = normalized(D.transpose() * tangent_normal); //切线空间法线转换到观察空间
        vec4 r = normalized(n * (n * light_dir)*2 - light_dir);   // reflected light direction
        double ambient = .4;                                      // ambient light intensity
        double diff = std::max(0., n * light_dir);                        // diffuse light intensity
        double spec = std::pow(std::max(r.z, 0.), 35)* 6*spec_intensity;                   // specular light intensity
        TGAColor gl_FragColor={255,255,255,255};     // output color of the fragment
        for(int channel:{0,1,2}){
            gl_FragColor[channel]=std::min<int>(255,diffuse_color[channel]* (ambient + diff + spec));
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
    texture_normal.read_tga_file("../obj/floor_nm_tangent.tga");
    texture_color.read_tga_file("../obj/floor_diffuse.tga");
    texture_spec.read_tga_file("../obj/floor_spec.tga");
    model Model("../obj/floor.obj");
     for(int i = 0; i < Model.nverts(); i++){
        vec3 v = Model.vert(i);
        vec4 view_pos = ModelView * vec4{v.x, v.y, v.z, 1.};
        vec4 clip_pos = Perspective * view_pos;
        std::cout << "Vertex " << i << ": " 
                  << "obj(" << v.x << "," << v.y << "," << v.z << ") "
                  << "view(" << view_pos.x << "," << view_pos.y << "," << view_pos.z << ") "
                  << "clip(" << clip_pos.x << "," << clip_pos.y << "," << clip_pos.z << "," << clip_pos.w << ")"
                  << std::endl;
    }
    
    PhongShader shader(light, Model);
   
    for(int i=0;i<Model.nfaces();i++){
        Triangle clip;
        for(int j=0;j<3;j++){
            clip[j] = shader.vertex(i,j);
        }
        rasterize(clip,shader,framebuffer);
    }
   
    framebuffer.write_tga_file("output.tga");
    TGAImage zbuffer_img(width, height, TGAImage::GRAYSCALE);

    //转换zbuffer为图像，归一化到0-255
    double zmin = *std::min_element(zbuffer.begin(), zbuffer.end());
    double zmax = *std::max_element(zbuffer.begin(), zbuffer.end());
    double z_second_min;
    std::vector<double> z_copy = zbuffer;
    std::sort(z_copy.begin(), z_copy.end());
    for(const double& z : z_copy){
            if(z > zmin){
                z_second_min = z;
                break;
            }
        }
    for(int x=0;x<width;x++){
        for(int y=0;y<height;y++){
            double zvalue = zbuffer[x + y * width];
            if(zvalue == std::numeric_limits<double>::max()){
                zbuffer_img.set(x, y, {0});
            } else {
                //线性归一化
                double gray = (255 * (zvalue - z_second_min) / (zmax - z_second_min));
                zbuffer_img.set(x, y, {static_cast<unsigned char>(gray)});
            }
        }
    }
    zbuffer_img.write_tga_file("zbuffer.tga");
    
    return 0;
}