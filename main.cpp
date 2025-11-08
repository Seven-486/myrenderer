#include <cstdlib>
#include "our_gl.hpp"
#include "Scene.hpp"

extern mat<4,4> Viewport,ModelView, Perspective,Modeltransform; // "OpenGL" state matrices and
extern std::vector<double> zbuffer;     // the depth buffer
extern std::vector<double> shadowbuffer; // depth buffer for shadow map from light source

constexpr int width  = 800;      // output image size
constexpr int height = 800;
constexpr vec3    eye{-1, 0, 2}; // camera position
constexpr vec3 center{ 0, 0, 0}; // camera direction
constexpr vec3     up{ 0, 1, 0}; // camera up vector
constexpr vec4 light{ 1, 1, 1, 0}; // light direction



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
        TGAColor diffuse_color= Model.get_texture_color().get(uv.x * Model.get_texture_color().width(), uv.y * Model.get_texture_color().height());
        double spec_intensity = Model.get_texture_spec().get(uv.x * Model.get_texture_spec().width(), uv.y * Model.get_texture_spec().height())[0] / 255.0;
        TGAColor tex_normal = Model.get_texture_normal().get(uv.x * Model.get_texture_normal().width(), uv.y * Model.get_texture_normal().height());
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
struct shadowShader : IShader{//阴影贴图着色器,不必处理
    const model& Model;
    vec4 light_dir;
    vec4 tri[3];  
    shadowShader(const vec4& light, const model& m) : Model(m), light_dir(light) {
               light_dir = normalized(ModelView * vec4{light.x, light.y, light.z, 0.});
    }
    virtual vec4 vertex(const int face, const int vert){
        vec3 v =Model.vert(face, vert);                          // current vertex in object coordinates
        vec4 gl_Position = ModelView * vec4{v.x, v.y, v.z, 1.};
        tri[vert] = gl_Position;
        return Perspective * gl_Position;                         // in clip coordinates
    }
    virtual std::pair<bool ,TGAColor> fragment(const vec3 bar)const {
        return {false, {255,255,255,255}};                                    // do not discard the pixel
    }
    
};
//转换zbuffer为图像，归一化到0-255
void zbuffer_to_image(const std::vector<double>& zbuffer, TGAImage& image){
      TGAImage zbuffer_img(width, height, TGAImage::GRAYSCALE);
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
    image = zbuffer_img;
}

int main(){
    init_modeltransform(1.0, {0,0,0}, {0,0,0});// --- IGNORE ---
    lookat(eye, center, up);                                   // build the ModelView   matrix
    init_perspective_simple(norm(eye-center));                        // build the Perspective matrix
    init_viewport(width/16, height/16, width*7/8, height*7/8); // build the Viewport    matrix
    init_zbuffer(width, height);
    TGAImage framebuffer(width, height, TGAImage::RGB,{177, 195, 209, 255});
    // texture_normal.read_tga_file("../obj/floor_nm_tangent.tga");
    // texture_color.read_tga_file("../obj/floor_diffuse.tga");
    // texture_spec.read_tga_file("../obj/floor_spec.tga");
    // model Model("../obj/floor.obj");
    Scene scene(800,800);
    model Model1("../obj/diablo3_pose/diablo3_pose.obj");
    Model1.read_normalmap("../obj/diablo3_pose/diablo3_pose_nm_tangent.tga");
    Model1.read_texture("../obj/diablo3_pose/diablo3_pose_diffuse.tga");
    Model1.read_specmap("../obj/diablo3_pose/diablo3_pose_spec.tga");
    model Model2("../obj/floor.obj");
    Model2.read_normalmap("../obj/floor_nm_tangent.tga");
    Model2.read_texture("../obj/floor_diffuse.tga");
    Model2.read_specmap("../obj/floor_spec.tga");
    scene.add_model(Model1);
    scene.add_model(Model2);
    //渲染场景
   for(int i=0;i<scene.models.size();i++){
       PhongShader p_shader(light, scene.models[i]);
       for(int j=0;j<scene.models[i].nfaces();j++){
           Triangle clip;
           for(int k=0;k<3;k++){
               clip[k] = p_shader.vertex(j,k);
           }
           rasterize(clip,p_shader,framebuffer);
       }
   }

    framebuffer.write_tga_file("output.tga");
    mat<4,4> M= (Viewport*Perspective*ModelView).invert(); //从屏幕空间到光源空间的变换矩阵

    std::vector<double> zbuffer1(zbuffer);
    TGAImage zbuffer1_img(width, height, TGAImage::GRAYSCALE);
    zbuffer_to_image(zbuffer1,zbuffer1_img);
    zbuffer1_img.write_tga_file("zbuffer1.tga");
    //注意要清空zbuffer
    init_zbuffer(width, height);
    TGAImage trash ={height, width, TGAImage::RGB,{177, 195, 209, 255}};
    lookat(light.xyz(), center, up);                                   // build the ModelView   matrix
    init_perspective_simple(norm(eye-center));                        // build the Perspective matrix
    init_viewport(width/16, height/16, width*7/8, height*7/8); // build the Viewport    matrix
    init_shadowbuffer(width, height);
    //渲染阴影贴图
   for(int i=0;i<scene.models.size();i++){
       shadowShader shader(light, scene.models[i]);
       for(int j=0;j<scene.models[i].nfaces();j++){
           Triangle clip;
           for(int k=0;k<3;k++){
               clip[k] = shader.vertex(j,k);
           }
           rasterize(clip,shader,trash);
       }
   }
   std::vector<double> zbuffer2(zbuffer); //保存阴影贴图的zbuffer
   mat<4,4> N = Viewport*Perspective*ModelView; //阴影贴图的变换矩阵
   //准备生成mask
   std::vector<bool> shadow_mask(width * height, false);
    for(int x=0;x<width;x++){
        for(int y=0;y<height;y++){
            vec4 fragment =M*vec4{double(x),double(y),zbuffer1[x + y * width],1.};
            vec4 q= N * fragment;
            vec3 shadow_coord = q.xyz()/q.w;
            bool in_shadow =(fragment.z<-100||
                            (shadow_coord.x<0||shadow_coord.x>=width||shadow_coord.y<0||shadow_coord.y>=height)||
                            (shadow_coord.z> zbuffer2[int(shadow_coord.x) + int(shadow_coord.y) * width]-.03));
            shadow_mask[x + y * width] = in_shadow;
        }
    }
    TGAImage shadow_mask_img(width, height, TGAImage::GRAYSCALE);
    for(int x=0;x<width;x++){
        for(int y=0;y<height;y++){
            if(shadow_mask[x + y * width]){
                shadow_mask_img.set(x, y, {255,255,255,255});
            } else {
                shadow_mask_img.set(x, y, {0,0,0,255});
            }
        }
    }
    shadow_mask_img.write_tga_file("shadow_mask.tga");
    //根据mask调整最终图像
    for(int x=0;x<width;x++){
        for(int y=0;y<height;y++){
           if(shadow_mask[x+y*width]) continue;
                TGAColor c = framebuffer.get(x, y); 
                vec3 a = {double(c[0]), double(c[1]), double(c[2])};
                if(norm(a)>80){
                    a=normalized(a)*80;
                    framebuffer.set(x, y, {static_cast<unsigned char>(a.x),static_cast<unsigned char>(a.y),static_cast<unsigned char>(a.z),255});
                }
                else continue;
        }
    }
    framebuffer.write_tga_file("output_with_shadow.tga");
    TGAImage zbuffer2_img(width, height, TGAImage::GRAYSCALE);
    zbuffer_to_image(zbuffer2,zbuffer2_img);
    zbuffer2_img.write_tga_file("zbuffer2.tga");
    return 0;
}