#include <iostream>
#include <cstring>
#include<chrono>
#include <cmath>
#include "model.hpp"
#include "tgaimage.hpp"
const int width  = 800;
const int height = 800;
constexpr vec3    eye{-1,0,2}; // camera position
constexpr vec3 center{0,0,0};  // camera direction
constexpr vec3     up{0,1,0};  // camera up vector
constexpr TGAColor white   = {255, 255, 255, 255}; // attention, BGRA order
constexpr TGAColor green   = {  0, 255,   0, 255};
constexpr TGAColor red     = {  0,   0, 255, 255};
constexpr TGAColor blue    = {255, 128,  64, 255};
constexpr TGAColor yellow  = {  0, 200, 255, 255};


mat<4,4> View, Viewport, Perspective, Modeltransform;
double signed_triangle_area(int ax, int ay, int bx, int by, int cx, int cy) { //计算有向面积
    return .5*((by-ay)*(bx+ax) + (cy-by)*(cx+bx) + (ay-cy)*(ax+cx));
}
void modeltransform(double scale_factor, vec3 translate_vector, vec3 rotate_vector) {

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
void lookat(const vec3 eye, const vec3 center, const vec3 up) { //视图变换矩阵
    vec3 n = normalized(eye-center);
    vec3 l = normalized(cross(up,n));
    vec3 m = normalized(cross(n, l));
    View = mat<4,4>{{{l.x,l.y,l.z,0}, {m.x,m.y,m.z,0}, {n.x,n.y,n.z,0}, {0,0,0,1}}} *
                mat<4,4>{{{1,0,0,-center.x}, {0,1,0,-center.y}, {0,0,1,-center.z}, {0,0,0,1}}};
}
void perspective(double fov=M_PI/3.0, double aspect=1.0f, double near=0.1f, double far=10.f) { //透视投影矩阵
    double f = 1.0 / std::tan(fov * 0.5);
    Perspective = {{{f/aspect,0,0,0}, {0,f,0,0}, {0,0,(far+near)/(near-far),(2*far*near)/(near-far)}, {0,0,-1,0}}};
}
//简单版透视投影矩阵
void perspective_simple(const double f ) {
   Perspective = {{{1,0,0,0}, {0,1,0,0}, {0,0,1,0}, {0,0, -1/f,1}}};
}
void viewport(const int x, const int y, const int w, const int h) { //视口变换矩阵
    Viewport = {{
        {w/2., 0, 0, x+w/2.}, 
        {0, h/2., 0, y+h/2.}, 
        {0,0,1,0}, 
        {0,0,0,1}}};
}

void rasterized_triangle(const vec4 clip[3],TGAImage &framebuffer, std::vector<double> &zbuffer, TGAColor color) {
    //实现透视除法
    vec4 ndc[3];
    for (int i=0; i<3; i++) {
        ndc[i] = clip[i] / clip[i].w;
    }
    //利用Viewport矩阵将裁剪空间坐标转换为屏幕坐标
    vec3 screen[3];
    double inv_w[3];
    for (int i=0; i<3; i++) {
        inv_w[i] = 1.0 / clip[i].w;
        vec4 vp = Viewport * vec4{ndc[i].x, ndc[i].y, ndc[i].z, 1};
        screen[i] = {vp.x, vp.y, vp.z};
    }
    //计算包围盒
    int min_x = std::max(0, (int)std::floor(std::min({screen[0].x, screen[1].x, screen[2].x})));
    int max_x = std::min(width - 1, (int)std::ceil(std::max({screen[0].x, screen[1].x, screen[2].x})));
    int min_y = std::max(0, (int)std::floor(std::min({screen[0].y, screen[1].y, screen[2].  y})));
    int max_y = std::min(height - 1, (int)std::ceil(std::max({screen[0].y, screen[1].y, screen[2].y})));
    //计算面积
    double area = signed_triangle_area(screen[0].x, screen[0].y, screen[1].x, screen[1].y, screen[2].x, screen[2].y);
    if (area < 1) return; //剔除退化
    //遍历包围盒内的像素
    for (int y = min_y; y <= max_y; y++) {
        for (int x = min_x; x <= max_x; x++) {
            //计算重心坐标
            double alpha = signed_triangle_area(x, y, screen[1].x, screen[1].y, screen[2].x, screen[2].y) / area;
            double beta = signed_triangle_area(screen[0].x, screen[0].y, x, y, screen[2].x, screen[2].y) / area;
            double gamma = signed_triangle_area(screen[0].x, screen[0].y, screen[1].x, screen[1].y, x, y) / area;
            //判断是否在三角形内和测试深度
            if (alpha >= 0 && beta >= 0 && gamma >= 0) {
                double inv_w_interp = alpha * inv_w[0] + beta * inv_w[1] + gamma * inv_w[2];
                double w_interp = 1.0 / inv_w_interp; // 恢复 w
                // 插值 z（透视校正）
                double z_interp = (alpha * screen[0].z * inv_w[0] + 
                                  beta * screen[1].z * inv_w[1] + 
                                  gamma * screen[2].z * inv_w[2]) * w_interp;
                int idx = x + y * width;
                if (zbuffer[idx] < z_interp) {
                    zbuffer[idx] = z_interp;
                    framebuffer.set(x, y, color);
                }
            }
        }
    }
}

vec3 persp(vec3 v) { //简单的透视投影，暂时不用
    constexpr double c = 3.;
    return v / (1-v.z/c);
}

void line(int ax, int ay, int bx, int by, TGAImage &framebuffer, TGAColor color) { //Bresenham直线算法
    bool steep = std::abs(ax-bx) < std::abs(ay-by);
    if(steep) {
        std::swap(ax, ay);
        std::swap(bx, by);
    }
    if (ax>bx) {
        std::swap(ax, bx);
        std::swap(ay, by);
    }
     float y = ay;
        for (int x=ax; x<=bx; x++) {
           float t = (x-ax)/(float)(bx-ax);
        int y = std::round( ay + (by-ay)*t );
        if (steep) 
            framebuffer.set(y, x, color);
        else
         framebuffer.set(x, y, color);
         y += (by-ay) / static_cast<float>(bx-ax);
    }


}
// 使用扫描线法填充三角形
void triangle_2D_scanline(int ax, int ay, int bx, int by, int cx, int cy, TGAImage &framebuffer, TGAColor color) { 
     if (ay>by) { std::swap(ax, bx); std::swap(ay, by); }
    if (ay>cy) { std::swap(ax, cx); std::swap(ay, cy); }
    if (by>cy) { std::swap(bx, cx); std::swap(by, cy); }

  int total_height = cy-ay;

    if (ay != by) { // if the bottom half is not degenerate
        int segment_height = by - ay;
        for (int y=ay; y<=by; y++) { // sweep the horizontal line from ay to by
            int x1 = ax + ((cx - ax)*(y - ay)) / total_height;
            int x2 = ax + ((bx - ax)*(y - ay)) / segment_height;
            for (int x=std::min(x1,x2); x<std::max(x1,x2); x++)  // draw a horizontal line
                framebuffer.set(x, y, color);
        }
    }
     if (by != cy) { // if the upper half is not degenerate
        int segment_height = cy - by;
        for (int y=by; y<=cy; y++) { // sweep the horizontal line from by to cy
            int x1 = ax + ((cx - ax)*(y - ay)) / total_height;
            int x2 = bx + ((cx - bx)*(y - by)) / segment_height;
            for (int x=std::min(x1,x2); x<std::max(x1,x2); x++)  // draw a horizontal line
                framebuffer.set(x, y, color);
        }
    }
}
// 使用重心坐标法填充三角形
void triangle_2D_weighted(int ax, int ay, int bx, int by, int cx, int cy, TGAImage &framebuffer, TGAColor color) {
    int max_y = std::max({ay, by, cy});
    int min_y = std::min({ay, by, cy});
    int max_x = std::max({ax, bx, cx});
    int min_x = std::min({ax, bx, cx});
    
    // Calculate triangle area for normalization
    int area = (bx - ax) * (cy - ay) - (cx - ax) * (by - ay);// 2 times the area of the triangle
    if (area < 1) return; // 进行剔除，避免除以零或负面积

    for (int y = min_y; y <= max_y; y++) {
        for (int x = min_x; x <= max_x; x++) {
            // Barycentric coordinates
            int w0 = (bx - cx) * (y - cy) - (by - cy) * (x - cx);
            int w1 = (cx - ax) * (y - ay) - (cy - ay) * (x - ax);
            int w2 = (ax - bx) * (y - by) - (ay - by) * (x - bx);
            
            // Check if point is inside triangle (same orientation for all weights)
            if ((w0 >= 0 && w1 >= 0 && w2 >= 0) || (w0 <= 0 && w1 <= 0 && w2 <= 0)) {
                framebuffer.set(x, y, color);
            }
        }
    }
}
// 使用重心坐标法填充三角形并插值颜色和深度
void triangle_2D(int ax, int ay, int bx, int by, int cx, int cy, TGAImage &framebuffer) {
    int max_y = std::max({ay, by, cy});
    int min_y = std::min({ay, by, cy});
    int max_x = std::max({ax, bx, cx});
    int min_x = std::min({ax, bx, cx});
    for (int y = min_y; y <= max_y; y++) {
        for (int x = min_x; x <= max_x; x++) {
            //重心坐标法判断点是否在三角形内
            int w0 = (bx - cx) * (y - cy) - (by - cy) * (x - cx); // area of sub-triangle PBC
            int w1 = (cx - ax) * (y - ay) - (cy - ay) * (x - ax);// area of sub-triangle PCA
            int w2 = (ax - bx) * (y - by) - (ay - by) * (x - bx);// area of sub-triangle ABP
            if (w0 >= 0 && w1 >= 0 && w2 >= 0) {
                //假设三点为红色，绿色，蓝色，则根据重心坐标插值颜色
                TGAColor color;
                color[2] = (w0 * 255) / (w0 + w1 + w2); // Red channel
                color[1] = (w1 * 255) / (w0 + w1 + w2); // Green channel
                color[0] = (w2 * 255) / (w0 + w1 + w2); // Blue channel
                color[3] = 255; // Alpha channel
                framebuffer.set(x, y, color);
            }
        }
    }
}
// 使用重心坐标法填充三角形并利用深度缓冲进行深度测试
void triangle_3D(int ax, int ay, int az, int bx, int by, int bz, int cx, int cy, int cz, TGAImage& framebuffer, TGAImage &zbuffer, TGAColor color) {
    int max_y = std::max({ay, by, cy});
    int min_y = std::min({ay, by, cy});
    int max_x = std::max({ax, bx, cx});
    int min_x = std::min({ax, bx, cx});
    
    // 预计算三角形面积
    double area = signed_triangle_area(ax, ay, bx, by, cx, cy);
    if (std::abs(area) < 1e-6) return; // Skip degenerate triangles
    
    for (int y = min_y; y <= max_y; y++) {
        for (int x = min_x; x <= max_x; x++) {
            // 边界检查
            if (x < 0 || x >= width || y < 0 || y >= height) continue;
            
            double alpha = signed_triangle_area(x, y, bx, by, cx, cy) / area;
            double beta = signed_triangle_area(ax, ay, x, y, cx, cy) / area;
            double gamma = signed_triangle_area(ax, ay, bx, by, x, y) / area;
            
            if (alpha >= 0 && beta >= 0 && gamma >= 0) {
                // 插值深度值
                double z = alpha * az + beta * bz + gamma * cz;
                unsigned char z_byte = static_cast<unsigned char>(std::max(0.0, std::min(255.0, z)));
                
                if (z_byte >= zbuffer.get(x, y)[0]) { // 修正深度测试条件
                    zbuffer.set(x, y, {z_byte});
                    framebuffer.set(x, y, color);
                }
            }
        }
    }
}
// 3D向量旋转函数，绕X、Y、Z轴旋转指定角度 未整合进模型变化
vec3 rot(vec3 v, double angle_x=0, double angle_y=0, double angle_z=0) {
    double rad_x = angle_x * M_PI / 180.0;
    double rad_y = angle_y * M_PI / 180.0;
    double rad_z = angle_z * M_PI / 180.0;

    // Rotation matrix around X axis
    mat<3,3> Rx = {{
        {1, 0, 0},
        {0, cos(rad_x), -sin(rad_x)},
        {0, sin(rad_x), cos(rad_x)}
    }};

    // Rotation matrix around Y axis
    mat<3,3> Ry = {{
        {cos(rad_y), 0, sin(rad_y)},
        {0, 1, 0},
        {-sin(rad_y), 0, cos(rad_y)}
    }};

    // Rotation matrix around Z axis
    mat<3,3> Rz = {{
        {cos(rad_z), -sin(rad_z), 0},
        {sin(rad_z), cos(rad_z), 0},
        {0, 0, 1}
    }};

    // Combined rotation matrix R = Rz * Ry * Rx
    mat<3,3> R = Rz * Ry * Rx;

    return R * v;
}

std::tuple<int,int,int> project(vec3 v){ //投影到屏幕空间,可以叫做视口变换
    int x= (v.x +1.)*width/2.;
    int y= (v.y +1.)*height/2.;
    int z= (v.z +1.)*255/2.;
    return {x,y,z};
}


int main(){
//线框图

//     model model("/Users/nasu/Documents/cpp_project/myrenderer/obj/diablo3_pose/diablo3_pose.obj");
//     for(int i=0;i<model.nfaces();i++){ //遍历每个面
//         //获取每个面的三个顶点
//         vec3 v0= model.vert(i,0);
//         vec3 v1= model.vert(i,1);
//         vec3 v2= model.vert(i,2);
//         //投影到屏幕空间
//         auto [x0,y0]= project(v0);
//         auto [x1,y1]= project(v1);
//         auto [x2,y2]= project(v2);
//         //绘制三角形
//         triangle_2D_weighted(x0,y0,x1,y1,x2,y2,framebuffer,{static_cast<unsigned char>(rand()%256),static_cast<unsigned char>(rand()%256),static_cast<unsigned char>(rand()%256),255});
//     }

//    framebuffer.write_tga_file("output.tga");
//    return 0;

// 填充三角形
//  TGAImage framebuffer(width, height, TGAImage::RGB);
//     triangle_weighted(7, 45, 35, 100, 45,  60, framebuffer, red);
//     triangle_weighted(120, 35, 90,   5, 45, 110, framebuffer, white);
//     triangle_weighted(115, 83, 80,  90, 85, 120, framebuffer, green);


//     framebuffer.write_tga_file("framebuffer.tga");
//     return 0;

//插值颜色三角形 默认红绿蓝顶点颜色
    // TGAImage framebuffer(width, height, TGAImage::RGB);
    //     triangle_2D(7, 45, 35, 100, 45,  60, framebuffer);
    //     triangle_2D(120, 35, 90,   5, 45, 110, framebuffer);
    //     triangle_2D(115, 83, 80,  90, 85, 120, framebuffer);
    //     framebuffer.write_tga_file("framebuffer.tga");
    //     return 0;

// 深度缓冲三角形
    TGAImage framebuffer(width, height, TGAImage::RGB);
    std::vector<double> zbuffer(width*height, -std::numeric_limits<double>::max());
    model model("../obj/african_head/african_head.obj");
    modeltransform(1.0, {0,0,0}, {0,0,0}); //模型变换：缩放1倍，平移0，旋转0度
    lookat(eye, center, up);                              // build the ModelView   matrix
    perspective_simple(norm(eye-center));                        // build the Perspective matrix
    viewport(width/16, height/16, width*7/8, height*7/8); // build the Viewport    matrix


    for(int i=0;i<model.nfaces();i++){ //遍历每个面
            vec4 clip[3];
    for(int d : {0,1,2}){
        vec3 v= model.vert(i,d);
        clip[d]= Perspective * View * Modeltransform * vec4{v.x,v.y,v.z,1.0};
    }
    TGAColor color= {static_cast<unsigned char>(rand()%256),static_cast<unsigned char>(rand()%256),static_cast<unsigned char>(rand()%256),255};
    //实现三角形渲染函数 
    rasterized_triangle(clip,framebuffer,zbuffer,color);
    }
    framebuffer.write_tga_file("framebuffer.tga");
    //把深度缓冲写入文件以便查看
    TGAImage zbuffer_image(width, height, TGAImage::GRAYSCALE);
    // 寻找深度缓冲的最小和最大值以进行归一化，最小值对应最远的点，最大值对应最近的点，最小值注意不能取默认的初始值
    //取次小值
    double min_depth = *std::min_element(zbuffer.begin(), zbuffer.end());
    double second_min_depth = std::numeric_limits<double>::max();
    for (double d : zbuffer) {
        if (d > min_depth && d < second_min_depth) {
            second_min_depth = d;
        }
    }
    double max_depth = *std::max_element(zbuffer.begin(), zbuffer.end());
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int idx = x + y * width;
            // Normalize depth value to 0-255 range
            double depth_value = 0;
            if (max_depth > second_min_depth) {
                depth_value = ((zbuffer[idx] - second_min_depth) / (max_depth - second_min_depth)) * 255;
            }
            zbuffer_image.set(x, y, {static_cast<unsigned char>(depth_value)});

        }
    }
    zbuffer_image.write_tga_file("zbuffer.tga");
    return 0;
}