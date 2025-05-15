#include "pointcloud.h" 
#include "octree.h"     
#include <bits/stdc++.h> 

int main(int argc, char* argv[]) {
    if (argc != 4) {
        std::cerr << "Cách dùng: ./encode <file_in> <error> <file_out>\n";
        return 1;
    }
    // Chuyển đổi tham số error (argv[2]) từ chuỗi thành số thực
    float error = std::stof(argv[2]);
    if (error <= 0 || error > 1) {
        std::cerr << "Sai tham số error, vui lòng nhập lại\n";
        return 1;
    }

    // Khởi tạo biến để đo thời gian thực thi
    struct timespec start, end;
    clock_gettime(CLOCK_MONOTONIC, &start);  // Tạo đối tượng pointclound để lưu đám mây điểm

    //QUANG::PointCloud test;
    QUANG::PointCloud test(argv[1], 0); // Tạo đối tượng pointclound để lưu đám mây điểm
    if (test.size == 0) {
        return 1;
    }
    QUANG::point max_bound = {-99999, -999999, -999999}, min_bound = {999999, 999999, 999999};
    for (int i = 0; i < test.size; i++) 
{   
    QUANG::point p = ((QUANG::point *)test.vertices)[i]; // Lấy tọa độ của điểm thứ i trong đám mây điểm.
    if (max_bound.x < p.x) max_bound.x = p.x; 
    if (max_bound.y < p.y) max_bound.y = p.y; 
    if (max_bound.z < p.z) max_bound.z = p.z; 
    if (min_bound.x > p.x) min_bound.x = p.x; 
    if (min_bound.y > p.y) min_bound.y = p.y; 
    if (min_bound.z > p.z) min_bound.z = p.z;
}
    QUANG::octree root(min_bound, max_bound);
    QUANG::from_pc_to_octree(&test, &root, error);
    QUANG::write_octree_to_file(&root, argv[3]);// Lưu cây octree vào file đầu ra (argv[3])

    clock_gettime(CLOCK_MONOTONIC, &end);
    double elapsed_time = (end.tv_sec - start.tv_sec) * 1000.0 +
                          (end.tv_nsec - start.tv_nsec) / 1e6;
    std::cout << elapsed_time << " ms" << std::endl;
    return 0;
    
}
