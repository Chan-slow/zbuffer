#include <iostream>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <limits>
#include "tgaimage.h"
#include "model.h"
#include "geometry.h"

const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red = TGAColor(255, 0, 0, 255);
Model* model = NULL;
const int width = 800;
const int height = 800;

void line(int x0, int y0, int x1, int y1, TGAImage& image, TGAColor color) {
    bool steep = false;
    if (std::abs(x0 - x1) < std::abs(y0 - y1)) {
        std::swap(x0, y0);
        std::swap(x1, y1);
        steep = true;
    }
    if (x0 > x1) {
        std::swap(x0, x1);
        std::swap(y0, y1);
    }

    for (int x = x0; x <= x1; x++) 
    {
        float t = (x - x0) / (float)(x1 - x0);
        int y = y0 * (1. - t) + y1 * t;
        if (steep) {
            image.set(y, x, color);
        }
        else {
            image.set(x, y, color);
        }
    }
}

Vec3f barycentric(Vec3f *pts, Vec3f p)
{
    Vec3f A = Vec3f(pts[0].x, pts[0].y, 0);
    Vec3f B = Vec3f(pts[1].x, pts[1].y, 0);
    Vec3f C = Vec3f(pts[2].x, pts[2].y, 0);

    Vec3f s[2];
    for (int i = 2; i--; )
    {
        s[i][0] = B[i] - A[i];
        s[i][1] = C[i] - A[i];
        s[i][2] = A[i] - p[i];
    }

    
    Vec3f m = cross(s[0], s[1]);
    
    if (std::abs(m[2]) > 1e-2)
        return Vec3f(1.f - (m.x + m.y) / m.z, m.x / m.z, m.y / m.z);
    return Vec3f(-1., 1., 1.);
}

void triangle(Vec3f* pts,float *zbuffer,TGAImage& image, TGAColor color)
{
    Vec2f boxmin (image.get_width() - 1, image.get_height() - 1);
    Vec2f boxmax(0, 0);
    Vec2f Clamp(image.get_width() - 1, image.get_height() - 1);

    for (int i = 0; i < 3; i++)
    {
        boxmin.x = std::max(0.f, std::min(pts[i].x, boxmin.x));
        boxmin.y = std::max(0.f, std::min(pts[i].y, boxmin.y));

        boxmax.x = std::min(Clamp.x, std::max(pts[i].x, boxmax.x));
        boxmax.y = std::min(Clamp.y, std::max(pts[i].y, boxmax.y));
    }
    Vec3f pi;
    for (pi.x = boxmin.x; pi.x < boxmax.x; pi.x++)
    {
        for (pi.y = boxmin.y; pi.y < boxmax.y; pi.y++)
        {
            Vec3f bc_Screen = barycentric(pts, pi);
            if (bc_Screen.x < 0 || bc_Screen.y < 0 || bc_Screen.z < 0) continue;
            pi.z = 0;
            for (int i = 0; i < 3; i++) pi.z += pts[i][2] * bc_Screen[i];
            if (zbuffer[int(pi.x + pi.y * width)] < pi.z) {
                zbuffer[int(pi.x + pi.y * width)] = pi.z;
                //TGAColor zb = TGAColor((pi.z+1) *0.5* 255, (pi.z + 1) * 0.5 * 255, (pi.z + 1) * 0.5 * 255, 255);
                image.set(pi.x, pi.y,color);
             
            }
            
        }
    }
}

Vec3f world2screen(Vec3f v) {
    return Vec3f(int((v.x + 1.) * width / 2. + .5), int((v.y + 1.) * height / 2. + .5), v.z);
}

int main(int argc, char** argv) {
    if (2 == argc)
    {
        model = new Model(argv[1]);
    }
    else
    {
        model = new Model("obj/african_head.obj");
    }
    TGAImage image(width, height, TGAImage::RGB);
    Vec3f light_dir(0, 0, -1); // define light_dir
    float *zbuffer = new float[width * height];
    for (int i = width * height; i--; zbuffer[i] = -std::numeric_limits<float>::max());

    for (int i = 0; i < model->nfaces(); i++)
    {
        std::vector<int>face = model->face(i); 
        Vec3f screen_coords[3];
        Vec3f world_coords[3];
        for (int j = 0; j < 3; j++)
        {
            Vec3f v = model->vert(face[j]);
            
            screen_coords[j] = screen_coords[j] = world2screen(model->vert(face[j]));
            world_coords[j] = v;
        }
        for (int k = 0; k < 3; k++)
        {
            
        }
        Vec3f n = cross(world_coords[2] - world_coords[0], world_coords[1] - world_coords[0]);
        n = n.normalize();
        float Lambert = std::max(0.f, light_dir * n);

        if (Lambert > 0)
        {   
            
            triangle(screen_coords,zbuffer, image, TGAColor(Lambert * 255, Lambert * 255, Lambert * 255, 255));
        }
    }

    
    image.flip_vertically(); // i want to have the origin at the left bottom corner of the image
    image.write_tga_file("output.tga");
    delete model;
    return 0;
}
