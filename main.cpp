#include <iostream>
#include <cstring>
#include<chrono>
#include "model.hpp"
#include "tgaimage.hpp"
const int width  = 800;
const int height = 800;
constexpr TGAColor white   = {255, 255, 255, 255}; // attention, BGRA order
constexpr TGAColor green   = {  0, 255,   0, 255};
constexpr TGAColor red     = {  0,   0, 255, 255};
constexpr TGAColor blue    = {255, 128,  64, 255};
constexpr TGAColor yellow  = {  0, 200, 255, 255};

double signed_triangle_area(int ax, int ay, int bx, int by, int cx, int cy) { //计算有向面积
    return .5*((by-ay)*(bx+ax) + (cy-by)*(cx+bx) + (ay-cy)*(ax+cx));
}
void line(int ax, int ay, int bx, int by, TGAImage &framebuffer, TGAColor color) {
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
std::tuple<int,int,int> project(vec3 v){ //投影到屏幕空间
    int x= (v.x +1.)*width/2.;
    int y= (v.y +1.)*height/2.;
    int z= (v.z +1.)*255/2.;
    return {x,y,z};
}

int main(){
    //线框图

    TGAImage framebuffer= TGAImage(width,height,TGAImage::RGB);
    TGAImage zbuffer= TGAImage(width,height,TGAImage::GRAYSCALE);

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
    model model("/Users/nasu/Documents/cpp_project/myrenderer/obj/diablo3_pose/diablo3_pose.obj");
    for(int i=0;i<model.nfaces();i++){ //遍历每个面
        //获取每个面的三个顶点
        vec3 v0= model.vert(i,0);
        vec3 v1= model.vert(i,1);
        vec3 v2= model.vert(i,2);
        //投影到屏幕空间
        auto [x0,y0,z0]= project(v0);
        auto [x1,y1,z1]= project(v1);
        auto [x2,y2,z2]= project(v2);
        //绘制三角形
        triangle_3D(x0,y0,z0,x1,y1,z1,x2,y2,z2,framebuffer,zbuffer,{static_cast<unsigned char>(rand()%256),static_cast<unsigned char>(rand()%256),static_cast<unsigned char>(rand()%256),255});
    }
    framebuffer.write_tga_file("framebuffer.tga");
    zbuffer.write_tga_file("zbuffer.tga");
    return 0;

}