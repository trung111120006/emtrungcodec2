#pragma once
#include <stdio.h>
#include <queue>
#include <bitset>
#include <pointcloud.h>
namespace QUANG
{

struct point {
    float x, y, z;
    point operator+(const point& other) const { return {x + other.x, y + other.y, z + other.z}; }
    point operator*(float scalar) const { return {x * scalar, y * scalar, z * scalar}; }
};

struct color {
    unsigned char r, g, b;
};

struct PointCloud {
    int size;
    int count;
    point* vertices;
    color* rgb;
};

struct octree {
    point min_bound, max_bound;
    color attribute;
    octree* children[8];
    int count; // Số nút lá trong cây con
    octree(const point& min_b, const point& max_b) : min_bound(min_b), max_bound(max_b), count(0) {
        for (int i = 0; i < 8; ++i) children[i] = nullptr;
    }
    ~octree() {
        for (int i = 0; i < 8; ++i) delete children[i];
    }
};

// Hàm tính khoảng cách bình phương giữa hai điểm
float get_d2(const point& p1, const point& p2);

// Hàm xác định octant của điểm so với trung tâm
unsigned char get_octant(const point& p, const point& center);

// Hàm chèn điểm vào octree
void insert_point(octree* node, const point& point, const color& attribute, float& error2);

// Hàm chuyển từ PointCloud sang octree
void from_pc_to_octree(PointCloud* pc, octree* root, float error);

// Hàm đếm số nút lá theo DFS
int count_leaves(octree* node);

// Hàm ghi octree vào luồng đầu ra
void write_octree(std::ofstream& out, octree* node);

// Hàm ghi octree ra file
void write_octree_to_file(octree* root, const char* filename);

// Hàm đọc octree từ luồng đầu vào
octree* read_octree(std::ifstream& in, const point& min_bound, const point& max_bound);

// Hàm tải octree từ file
octree* load_octree_from_file(const char* filename);

} // namespace QUANG
