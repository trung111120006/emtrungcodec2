#include "octree.h"     
#include "pointcloud.h"    
#include <thread>       
#include <mutex>        
#include <atomic>       
#include <condition_variable>    
#include <bits/stdc++.h>  

struct param {
    QUANG::PointCloud* pc; 
    QUANG::octree* root;   
    std::mutex* pc_mutex; // Mutex để bảo vệ đám mây điểm
};

// Lớp ThreadPool để quản lý các luồng tái sử dụng
class ThreadPool {
private:
    std::vector<std::thread> workers; 
    std::queue<std::function<void()>> tasks;
    std::mutex queue_mutex; // Mutex bảo vệ hàng đợi
    std::condition_variable condition; // Biến điều kiện để đồng bộ luồng
    std::atomic<bool> stop; // Cờ dừng ThreadPool

public:
    // Constructor: khởi tạo ThreadPool với số luồng chỉ định
    ThreadPool(size_t threads) : stop(false) {
        for (size_t i = 0; i < threads; ++i) {
            workers.emplace_back([this] {
                while (true) {
                    std::function<void()> task;
                    {
                        std::unique_lock<std::mutex> lock(queue_mutex);
                        condition.wait(lock, [this] { return stop || !tasks.empty(); });
                        if (stop && tasks.empty()) return; 
                        task = std::move(tasks.front()); 
                        tasks.pop();
                    }
                    task(); 
                }
            });
        }
    }

    // Destructor: dừng và chờ tất cả luồng
    ~ThreadPool() {
        {
            std::unique_lock<std::mutex> lock(queue_mutex);
            stop = true; // Đặt cờ dừng
        }
        condition.notify_all(); // Thông báo tất cả luồng
        for (std::thread& worker : workers) {
            worker.join(); // Chờ luồng hoàn thành
        }
    }

    // Thêm tác vụ vào hàng đợi
    void enqueue(std::function<void()> task) {
        {
            std::unique_lock<std::mutex> lock(queue_mutex);
            tasks.emplace(std::move(task)); // Thêm tác vụ
        }
        condition.notify_one(); // Thông báo một luồng
    }
};

// Hàm thêm điểm và màu vào đám mây điểm, thread-safe
void pc_addpointandrgb(QUANG::PointCloud* pc, QUANG::point& p, QUANG::color& c, std::mutex* pc_mutex) {
    std::lock_guard<std::mutex> lock(*pc_mutex);
    if (pc->count < pc->size) {
        pc->vertices[pc->count * 3 + 0] = p.x; 
        pc->vertices[pc->count * 3 + 1] = p.y;
        pc->vertices[pc->count * 3 + 2] = p.z;
        pc->rgb[pc->count * 3 + 0] = c.r;
        pc->rgb[pc->count * 3 + 1] = c.g;
        pc->rgb[pc->count * 3 + 2] = c.b;
        pc->count++;
    }
}

// Hàm search: duyệt cây octree bằng DFS để tạo đám mây điểm
void search(param* p) {
    QUANG::PointCloud* pc = p->pc; 
    QUANG::octree* root = p->root; 
    std::mutex* pc_mutex = p->pc_mutex; 
    
    std::vector<QUANG::octree*> stack;
    stack.reserve(1024); // Dự trữ không gian để tránh tái cấp phát
    stack.push_back(root); // Thêm nút gốc vào stack

    // Duyệt cây octree theo DFS
    while (!stack.empty()) {
        QUANG::octree* node = stack.back(); // Lấy nút cuối stack
        stack.pop_back(); // Xóa nút khỏi stack

        bool has_child = false; // Cờ kiểm tra nút có con
        // Duyệt ngược nút con (7->0) để tối ưu bộ nhớ cache
        for (int i = 7; i >= 0; --i) {
            if (node->children[i]) {
                has_child = true; // Đánh dấu nếu có nút con
                stack.push_back(node->children[i]); // Thêm nút con vào stack
            }
        }

        // Nếu là nút lá, thêm điểm vào đám mây điểm
        if (!has_child) {
            QUANG::point mid = (node->min_bound + node->max_bound) * 0.5f; // Tính điểm giữa
            pc_addpointandrgb(pc, mid, node->attribute, pc_mutex); // Thêm điểm và màu
        }
    }
}

void write_octree_to_ply(QUANG::octree* root, const char* filename, int n_thread) {
    if (!root) {
        std::cerr << "Lỗi: root là null\n";
        return;
    }
    // Khởi tạo đám mây điểm với bộ nhớ cấp phát trước
    QUANG::PointCloud* pc = new QUANG::PointCloud();
    pc->size = root->count; 
    pc->count = 0; 
    pc->vertices = new float[pc->size]; // Cấp phát mảng điểm
    pc->rgb = new unsigned char[pc->size]; // Cấp phát mảng màu
    std::mutex pc_mutex; // Mutex bảo vệ đám mây điểm

    // Xử lý đa luồng hoặc đơn luồng
    if (n_thread > 1) {
        ThreadPool pool(std::min(n_thread, 8)); 
        std::vector<param> params(8); 

        // Phân phối công việc cho các nút con
        for (int i = 0; i < 8; i++) {
            if (root->children[i]) {
                params[i] = {pc, root->children[i], &pc_mutex}; 
                pool.enqueue([&params, i] { search(&params[i]); }); 
            }
        }
    } else {
        // Chạy đơn luồng nếu n_thread <= 1
        param p{pc, root, &pc_mutex};
        search(&p);
    }

    write_pc_to_ply(pc, filename);

    delete[] pc->vertices;
    delete[] pc->rgb;
    delete pc;
}

int main(int argc, char* argv[]) {
    if (argc != 4) {
        std::cerr << "Cách dùng: ./decode <file_in> <n_threads> <file_out>\n";
        return 1;
    }

    // Chuyển đổi số luồng từ chuỗi thành số nguyên
    int n_threads;
    try {
        n_threads = std::stoi(argv[2]);
    } catch (const std::exception& e) {
        std::cerr << "Lỗi: n_threads phải là số nguyên\n";
        return 1;
    }
    if (n_threads <= 0) {
        std::cerr << "Số luồng phải lớn hơn 0\n";
        return 1;
    }
    struct timespec start_time, end_time;
    clock_gettime(CLOCK_REALTIME, &start_time);

    // Tải cây octree từ file
    QUANG::octree* octree_root = QUANG::load_octree_from_file(argv[1]);
    if (!octree_root) {
        std::cerr << "Không thể tải octree từ file: " << argv[1] << std::endl;
        return 1;
    }

    write_octree_to_ply(octree_root, argv[3], n_threads);

    delete octree_root;
    clock_gettime(CLOCK_REALTIME, &end_time);
    double elapsed_time_ms =
        (end_time.tv_sec - start_time.tv_sec) * 1000.0 +
        (end_time.tv_nsec - start_time.tv_nsec) / 1000000.0;

    std::cout << "Thời gian thực thi: " << elapsed_time_ms << " ms" << std::endl;

    return 0;