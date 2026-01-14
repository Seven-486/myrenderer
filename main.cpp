#include <cstdlib>
#include "our_gl.hpp"
#include "Scene.hpp"
#include<random>
#include<cmath>


double smoothstep(double edge0, double edge1, double x) {
    double t = std::clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0); //作用是将值限制在0到1之间
    return t * t * (3.0 - 2.0 * t); // 插值计算
}

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
        //以下进行双线性插值采样纹理颜色
        auto bilinear_sample = [](const TGAImage& texture, vec2 uv) -> TGAColor {
        int tex_width = texture.width();
        int tex_height = texture.height();
        
        // 钳制UV坐标
        double u_clamped = std::clamp(uv.x, 0.0, 0.999999);
        double v_clamped = std::clamp(uv.y, 0.0, 0.999999);
        
        // 计算像素坐标
        double u_float = u_clamped * tex_width;
        double v_float = v_clamped * tex_height;
        
        int u0 = static_cast<int>(u_float);
        int v0 = static_cast<int>(v_float);
        int u1 = std::min(u0 + 1, tex_width - 1);
        int v1 = std::min(v0 + 1, tex_height - 1);
        
        // 插值权重
        double s = u_float - u0;
        double t = v_float - v0;
        
        // 获取四个角的颜色
        TGAColor c00 = texture.get(u0, v0);
        TGAColor c10 = texture.get(u1, v0);
        TGAColor c01 = texture.get(u0, v1);
        TGAColor c11 = texture.get(u1, v1);
        
        // 双线性插值
        TGAColor result;
        for(int channel : {0, 1, 2, 3}) {
            result[channel] = static_cast<std::uint8_t>(
                (1 - s) * (1 - t) * c00[channel] +
                s * (1 - t) * c10[channel] +
                (1 - s) * t * c01[channel] +
                s * t * c11[channel]
            );
        }
        return result;
    };
        // 纹理采样
        TGAColor diffuse_color = bilinear_sample(Model.get_texture_color(), uv);
        
        //TGAColor diffuse_color= Model.get_texture_color().get(uv.x * Model.get_texture_color().width(), uv.y * Model.get_texture_color().height());
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
void poissonDiskSamples(const vec2& randomseed,std::vector<vec2>& samples){
    return ;
}




int main(){
    init_modeltransform(1.0, {0,0,0}, {0,0,0});// --- IGNORE ---
    lookat(eye, center, up);                                   // build the ModelView   matrix
    init_perspective_simple(norm(eye-center));                        // build the Perspective matrix
    init_viewport(width/16, height/16, width*7/8, height*7/8); // build the Viewport    matrix
    init_zbuffer(width, height);
    TGAImage framebuffer(width, height, TGAImage::RGB,{177, 195, 209, 255});
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
   
    framebuffer.write_tga_file("output_msaa4x.tga");
    std::vector<double> zbuffer1(zbuffer); //保存场景渲染的zbuffer
    
    // TGAImage zbuffer1_img(width, height, TGAImage::GRAYSCALE);
    // zbuffer_to_image(zbuffer1,zbuffer1_img);
    // zbuffer1_img.write_tga_file("zbuffer1.tga");
    /*  以下为暴力计算AO的代码
    由于每次都要重新渲染场景的阴影贴图，计算量巨大，运行时间较长
    这里使用了500次采样，可以根据需要调整采样次数以平衡质量和性能
    计算结果保存在final_mask_ao中
    std::vector<double> final_mask_ao(width*height,0); //保存最终的AO遮罩
    mat<4,4> M= (Viewport*Perspective*ModelView).invert(); //从屏幕空间到光源空间的变换矩阵
    constexpr int n=500;
    std::mt19937 gen((std::random_device())());
    std::uniform_real_distribution<double> dist(0.0, 1.0);

    
    for(int i=0;i<n;i++){ //多次采样计算阴影
        std::cout<<i<<" "<<std::endl;
     double y= dist(gen);
     double theta = 2.0*M_PI*dist(gen);
     double r= std::sqrt(1.0-y*y);
     vec3 light_random =vec3{r*std::cos(theta),y,r*std::sin(theta)}*1.5; //在半径为1.5的球面上采样光源位置
        //注意要清空zbuffer
    init_zbuffer(width, height);
    TGAImage trash ={height, width, TGAImage::RGB,{177, 195, 209, 255}};
    lookat(light_random, center, up);                                   // build the ModelView   matrix
    //init_perspective_simple(norm(eye-center));                        // build the Perspective matrix
    //init_viewport(width/16, height/16, width*7/8, height*7/8); // build the Viewport    matrix
    //init_shadowbuffer(width, height);
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
    // TGAImage zbuffer2_img(width, height, TGAImage::GRAYSCALE);
    // zbuffer_to_image(zbuffer,zbuffer2_img);
    // zbuffer2_img.write_tga_file("zbuffer2_sample_"+std::to_string(i)+".tga");
   mat<4,4> N = Viewport*Perspective*ModelView; //阴影贴图的变换矩阵
   
    for(int x=0;x<width;x++){
        for(int y=0;y<height;y++){
            vec4 fragment =M*vec4{double(x),double(y),zbuffer1[x + y * width],1.};
            vec4 q= N * fragment;
            vec3 shadow_coord = q.xyz()/q.w;
            double lit =(fragment.z<-100||
                            (shadow_coord.x>=0&&shadow_coord.x<width&&shadow_coord.y>=0&&shadow_coord.y<height&&
                            (shadow_coord.z> zbuffer[int(shadow_coord.x) + int(shadow_coord.y) * width]-.03)));
            final_mask_ao[x + y * width] += (lit-final_mask_ao[x + y * width])/(i+1.0); //增量平均
        }
    }
}

    for(int x=0;x<width;x++){
        for(int y=0;y<height;y++){
            double m=smoothstep(-1,1.0,final_mask_ao[x + y * width]);
            TGAColor c= framebuffer.get(x,y);
            framebuffer.set(x,y,{static_cast<unsigned char>(c[0]*m),static_cast<unsigned char>(c[1]*m),static_cast<unsigned char>(c[2]*m),c[3]});
        }
    }
    
    

    //根据final_ssao调整最终图像
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
    */
    //阴影映射部分
    std::vector<double> shadow_mask(width*height,0.0); //保存阴影遮罩
    mat<4,4> M= (Viewport*Perspective*ModelView).invert(); //从屏幕空间到光源空间的变换矩阵
    lookat (light.xyz(), center, up);                       // build the ModelView   matrix
    mat<4,4> Light_Matrix= Viewport*Perspective*ModelView; 
    //渲染阴影贴图
    //注意要清空zbuffer
    init_zbuffer(width, height);
    TGAImage trash ={height, width, TGAImage::RGB,{177, 195, 209, 255}};
    
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
   TGAImage zbuffer_shadow_img(width, height, TGAImage::GRAYSCALE);
   zbuffer_to_image(zbuffer,zbuffer_shadow_img);
   zbuffer_shadow_img.write_tga_file("shadow_zbuffer_pcf.tga");
   TGAImage shadow_mask_img_raw(width, height, TGAImage::GRAYSCALE);
   zbuffer_to_image(zbuffer1,shadow_mask_img_raw);
    shadow_mask_img_raw.write_tga_file("shadow_zbuffer_pcf2.tga");

    for(int x=0;x<width;x++){
        for(int y=0;y<height;y++){
            vec4 fragment =M*vec4{double(x),double(y),zbuffer1[x + y * width],1.}; //从场景渲染的zbuffer恢复片段位置
            vec4 q= Light_Matrix * fragment; //变换到光源裁剪空间
            vec3 shadow_coord = q.xyz()/q.w; //齐次除法得到光源屏幕空间坐标
            
            double shadow_intensity =0.0;
            double total_samples=0.0;
            int pcf_radius=3;
            double bias =0.03;

            if(fragment.z<-100||//background
               (shadow_coord.x<0 || shadow_coord.x>=width| shadow_coord.y<0 || shadow_coord.y>=height)   //out of light's view
               //||(shadow_coord.z> zbuffer[int(shadow_coord.x) + int(shadow_coord.y) * width]-.03)
               )
                { 
                shadow_intensity=1.0;
                total_samples=1.0;
                }
            else { 
                for(int dx=-pcf_radius;dx<=pcf_radius;dx++){
                    for(int dy=-pcf_radius;dy<=pcf_radius;dy++){
                        total_samples+=1.0;  
                        int nx=int(shadow_coord.x)+dx; 
                        int ny=int(shadow_coord.y)+dy;
                        if(nx >= 0 && nx < width && ny >= 0 && ny < height) {
                             double map_z = zbuffer[nx+ny*width];
                             if(shadow_coord.z > (map_z -bias)) { //visible
                                 shadow_intensity += 1.0;
                             }
                        }
                        else {
                            shadow_intensity += 1.0; //out of light's view treated as lit
                        }
                        
                    }
                }
            }

            // bool in_shadow =(fragment.z<-100||
            //                 (shadow_coord.x<0 || shadow_coord.x>=width|| shadow_coord.y<0 || shadow_coord.y>=width) || 
            //                 (shadow_coord.z> zbuffer[int(shadow_coord.x) + int(shadow_coord.y) * width]-.03));
            // shadow_mask[x + y * width] += in_shadow; 
            
            shadow_mask[x + y * width] =1.0-shadow_intensity/total_samples; 
        }     
    }
    TGAImage shadow_mask_img(width, height, TGAImage::GRAYSCALE);
    for(int x=0;x<width;x++){
        for(int y=0;y<height;y++){
            //线性归一化
            double gray = (255 * (1.0-shadow_mask[x + y * width]));//
            shadow_mask_img.set(x, y, {static_cast<unsigned char>(gray)});//
        }
    }
    shadow_mask_img.write_tga_file("shadow_pcf_mask.tga");
    //根据shadow_mask调整图像
    for(int x=0;x<width;x++){
        for(int y=0;y<height;y++){
               if(abs(shadow_mask[x+y*width])< 0.0001) continue; //跳过非阴影区域 
                TGAColor c = framebuffer.get(x, y); 
                vec3 a = {double(c[0]), double(c[1]), double(c[2])}; //原颜色
                double shadow_strength = 0.7; // 阴影浓度 0.0~1.0
                double shadow_factor = 1.0-shadow_mask[x + y * width] * shadow_strength; 
                a = a * shadow_factor;
                framebuffer.set(x, y, {
                    static_cast<unsigned char>(std::min(255.0, a.x)),
                    static_cast<unsigned char>(std::min(255.0, a.y)),
                    static_cast<unsigned char>(std::min(255.0, a.z)),
                    255
                });
            
            
                // if(norm(a)>80){ //限制最大亮度
                //     a=normalized(a)*80; //这是为了防止过曝
                //     framebuffer.set(x, y, {static_cast<unsigned char>(a.x),static_cast<unsigned char>(a.y),static_cast<unsigned char>(a.z),255});
                // }
                // else continue;
        }
    }


    //ssao的部分
    std::vector<double> final_mask_ssao(width*height,0); //保存最终的SSAO遮罩 
    constexpr int m=120; //采样次数
    const double radius=0.1; //采样半径
    std::mt19937 gen_ssao((std::random_device())()); //随机数生成器
    std::uniform_real_distribution<double> dist_ssao(-radius, radius); //均匀分布在[-radius, radius]之间

    for(int x=0;x<width;x++){
        for(int y=0;y<height;y++){
            double z= zbuffer1[x + y * width];
            if(z<-100) continue; //跳过背景
            vec4 fragement= Viewport.invert() * vec4{double(x),double(y),z,1.};//屏幕空间到观察空间
            double vote=0; //当前像素的投票值
            double voters=0; //当前像素的投票数
            for(int i=0;i<m;i++){
                vec4 p= Viewport *vec4{fragement+vec4{dist_ssao(gen_ssao),dist_ssao(gen_ssao),dist_ssao(gen_ssao),0.0}}; //采样点转换到屏幕空间
             if(p.x<0||p.x>=width||p.y<0||p.y>=height) continue; //跳过屏幕外采样点
             double sampled_z= zbuffer1[int(p.x) + int(p.y)*width]; //采样点的深度值
             if(sampled_z<-100) continue; //跳过背景采样点
             if(z+5*radius<sampled_z)continue; //采样点深度过大，跳过
                voters++; //有效投票数加一
                vote+=sampled_z>p.z; //如果采样点被遮挡，投一票
            }
            double ssao =smoothstep(0, 1, 1 - vote/voters*.4); //计算当前像素的ssao值
            TGAColor c= framebuffer.get(x,y);
            framebuffer.set(x,y,{static_cast<unsigned char>(c[0]*ssao),static_cast<unsigned char>(c[1]*ssao),static_cast<unsigned char>(c[2]*ssao),c[3]}); //调整颜色
        }
    }


    framebuffer.write_tga_file("output_with_pcfshadow_ssao120_binary.tga");
    //TGAImage zbuffer2_img(width, height, TGAImage::GRAYSCALE);
    //zbuffer_to_image(zbuffer2,zbuffer2_img);
    //zbuffer2_img.write_tga_file("zbuffer2.tga");
    return 0;
}