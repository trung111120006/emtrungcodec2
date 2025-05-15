#pragma once

#include <iostream>
#include <miniply.h>
#include <fstream>
#include <octree.h>

namespace QUANG
{
    int count_verts_ply(char *filename);
    bool load_point_cloud_ply(char *filename, float *pos, unsigned char *color);
    using point = myvector3<float>;
    using color = myvector3<unsigned char>;

    class PointCloud
    {
    public:
        float *vertices; 
        unsigned char *rgb;
        int size;
        int count;
        void deallocate();
        void read(const char *path, bool isRGBA);

    public:
        QUANG::point *Get_Vertices() { return reinterpret_cast<QUANG::point *>(vertices); }
        QUANG::color *Get_Rgb() { return reinterpret_cast<QUANG::color *>(rgb); }

        PointCloud() : vertices(nullptr), rgb(nullptr), size(0), count(0) {}
        PointCloud(const char *path, bool isRGBA = 1) { read(path, isRGBA); }
        ~PointCloud() { deallocate(); }

        void init(const char *path, bool isRGBA = 1);
    };
    void write_pc_to_ply(QUANG::PointCloud *pc, const char *outpath);
    void from_pc_to_octree(QUANG::PointCloud *pc, octree *root, float error);
}