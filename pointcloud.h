#pragma once

#include <miniply.h>
#include <fstream>
#include <iostream>

namespace QUANG
{
    int count_verts_ply(char *filename);
    bool load_point_cloud_ply(char *filename, float *pos, unsigned char *color);
    class PointCloud
    {
    public:
        QUANG::point *vertices; 
        QUANG::color *rgb;
        int size;
        int count;
        void deallocate();
        void read(const char *path, bool isRGBA);

    public:
        float *Get_Vertices() { return vertices; }
        unsigned char *Get_Rgb() { return rgb; }

        PointCloud() : vertices(nullptr), rgb(nullptr), size(0), count(0) {}
        PointCloud(const char *path, bool isRGBA = 1) { read(path, isRGBA); }
        ~PointCloud() { deallocate(); }

        void init(const char *path, bool isRGBA = 1);
    };
    void write_pc_to_ply(QUANG::PointCloud *pc, const char *outpath);
}