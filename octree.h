#pragma once
#include <stdio.h>
#include <queue>
#include <bitset>
#include <pointcloud.h>
namespace QUANG
{
    template <class T>
    struct myvector3
    {
        union
        {
            struct
            {
                T x, y, z;
            };
            struct
            {
                T r, g, b;
            };
        };

        myvector3(T x, T y, T z) : x(x), y(y), z(z) {}
        myvector3() : x(0), y(0), z(0) {}
        myvector3<T> &operator+=(const myvector3<T> &other)
        {
            x += other.x;
            y += other.y;
            z += other.z;
            return *this;
        }
        myvector3<T> &operator-=(const myvector3<T> &other)
        {
            x -= other.x;
            y -= other.y;
            z -= other.z;
            return *this;
        }
        bool operator==(const myvector3<T> &other) const { return x == other.x && y == other.y && z == other.z; }
        bool operator!=(const myvector3<T> &other) const { return !(*this == other); }
    };
    typedef myvector3<float> point;
    typedef myvector3<unsigned char> color;
    template <class T>
    myvector3<T> operator+(const myvector3<T> &a, const myvector3<T> &b) { return myvector3<T>(a.x + b.x, a.y + b.y, a.z + b.z); }
    template <class T>
    myvector3<T> operator-(const myvector3<T> &a, const myvector3<T> &b) { return myvector3<T>(a.x - b.x, a.y - b.y, a.z - b.z); }
    template <class T, class U>
    myvector3<T> operator*(const myvector3<T> &vec, const U &scalar) { return myvector3<T>(vec.x * scalar, vec.y * scalar, vec.z * scalar); }
    template <class T, class U>
    myvector3<T> operator*(const U &scalar, const myvector3<T> &vec) { return vec * scalar; }
    struct octree
    {
        color attribute;
        struct octree *children[8];
        QUANG::point min_bound, max_bound;
        int count;
        octree(const QUANG::point &min_bound, const QUANG::point &max_bound)
            : min_bound(min_bound), max_bound(max_bound), attribute{0, 0, 0}, count(0)
        {
            for (int i = 0; i < 8; ++i)
            {
                children[i] = nullptr;
            }
        }
        ~octree()
        {
            for (int i = 0; i < 8; ++i)
            {
                delete children[i];
            }
        }
    };
    inline unsigned char get_octant(const QUANG::point &point, const QUANG::point &center)
    {
        unsigned char octant = 0;
        if (point.x > center.x)
            octant |= 4;
        if (point.y > center.y)
            octant |= 2;
        if (point.z > center.z)
            octant |= 1;
        return octant;
    }
    inline float get_d2(const QUANG::point &a, const QUANG::point &b)
    {
        return (a.x - b.x) * (a.x - b.x) +
               (a.y - b.y) * (a.y - b.y) +
               (a.z - b.z) * (a.z - b.z);
    }
    void insert_point(octree *node, const QUANG::point &point, const QUANG::color &attribute, float &error2);
    void from_pc_to_octree(QUANG::PointCloud *pc, QUANG::octree &root, float error);
    void write_octree(std::ofstream &out, octree *node);
    void write_octree_to_file(octree *root, const char *filename);
    octree *read_octree(std::ifstream &in, const QUANG::point &min_bound, const QUANG::point &max_bound);
    octree *load_octree_from_file(const char *filename);
    int octree_diff(octree** diff, int* position, octree* ref, octree* pred);
}