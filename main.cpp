#include <iostream>
#include <cstring>
#include<chrono>
#include "tgaimage.hpp"

constexpr TGAColor white   = {255, 255, 255, 255}; // attention, BGRA order
constexpr TGAColor green   = {  0, 255,   0, 255};
constexpr TGAColor red     = {  0,   0, 255, 255};
constexpr TGAColor blue    = {255, 128,  64, 255};
constexpr TGAColor yellow  = {  0, 200, 255, 255};

void line(int ax,int ay,int bx, int by, TGAImage &framebuffer, TGAColor color){
    bool steep = std::abs(ax-bx)<std::abs(ay-by);
    if(steep){
        std::swap(ax,ay);
        std::swap(bx,by);
    }
    if(ax>bx){
        std::swap(ax,bx);
        std::swap(ay,by);
    }
    float y= ay;
    int ierror =0;
   for(int x=ax;x<=bx;x++){
         
         if(steep){
                framebuffer.set(y,x,color);
         }else{
                framebuffer.set(x,y,color);
            ierror += 2 * std::abs(by-ay);
       y += (by > ay ? 1 : -1) * (ierror > bx - ax);
        ierror -= 2 * (bx-ax)   * (ierror > bx - ax);
         }
   }

}

int main(int argc, char** argv){
    constexpr int width  = 64;
    constexpr int height = 64;
    TGAImage framebuffer(width, height, TGAImage::RGB);

    // int ax =  7, ay =  3;
    // int bx = 12, by = 37;
    // int cx = 62, cy = 53;

    // line(ax, ay, bx, by, framebuffer, blue);
    // line(cx,cy, bx, by, framebuffer, green);
    // line(cx,cy, ax, ay, framebuffer, yellow);
    // line(ax, ay, cx, cy, framebuffer, red);

    // framebuffer.set(ax, ay, white);
    // framebuffer.set(bx, by, white);
    // framebuffer.set(cx, cy, white);
     auto start = std::chrono::high_resolution_clock::now();
     std::srand(std::time({}));
    for (int i=0; i<(1<<24); i++) {
        int ax = rand()%width, ay = rand()%height;
        int bx = rand()%width, by = rand()%height;
        line(ax, ay, bx, by, framebuffer, { static_cast<unsigned char>(rand()%255), static_cast<unsigned char>(rand()%255), static_cast<unsigned char>(rand()%255), static_cast<unsigned char>(rand()%255) });
    }
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> diff = end - start;
    std::cout << "Time taken: " << diff.count() << " s\n";
    
    
    framebuffer.write_tga_file("framebuffer.tga");
    return 0;
}