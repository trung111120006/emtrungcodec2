#include "octree.h"

void QUANG::insert_point(octree *node, const QUANG::point &point, const QUANG::color &attribute, float &error2)
{
    if (!node)
        return;
    QUANG::point center = (node->min_bound + node->max_bound) * 0.5f;
    if (get_d2(point, center) <= error2)
    {
        node->attribute = attribute;
        return;
    }
    unsigned char octant = get_octant(point, center);
    if (!node->children[octant])
    {
        QUANG::point new_min = node->min_bound;
        QUANG::point new_max = center;
        if (octant & 4)
            new_min.x = center.x, new_max.x = node->max_bound.x;
        if (octant & 2)
            new_min.y = center.y, new_max.y = node->max_bound.y;
        if (octant & 1)
            new_min.z = center.z, new_max.z = node->max_bound.z;
        node->children[octant] = new octree(new_min, new_max);
    }
    insert_point(node->children[octant], point, attribute, error2);
}

void QUANG::from_pc_to_octree(QUANG::PointCloud *pc, octree *root, float error)
{
    float error2 = error * error;
    if (!pc || !root)
        return;
    for (int i = 0; i < pc->size; i++)
        insert_point(root, ((QUANG::point *)pc->vertices)[i], ((QUANG::color *)pc->rgb)[i], error2);
}
// Hàm đếm nút lá theo DFS
int count_leaves(octree* node) {
    if (!node) return 0;

    bool has_child = false;
    for (int i = 0; i < 8; ++i) {
        if (node->children[i]) {
            has_child = true;
            break;  // Nếu tìm thấy con, dừng kiểm tra.
        }
    }

    // Nếu không có con, node này là lá.
    if (!has_child) return 1;

    int count = 0;
    for (int i = 0; i < 8; ++i) {  // Duyệt từ 0 đến 7
        if (node->children[i]) {
            count += count_leaves(node->children[i]);
        }
    }

    return count;
}
void QUANG::write_octree(std::ofstream &out, octree *node)
{
    if (!node)
        return;
    unsigned char child_info = 0x00;
    for (int i = 0; i < 8; ++i)
        if ((node->children[i] != nullptr))
            child_info |= (1 << i);
    out.write((char *)(&child_info), sizeof(unsigned char));
    if (!child_info)
        out.write((char *)&node->attribute, sizeof(QUANG::color));
    // Duyệt các nút con theo thứ tự từ 7->0 
    for (int i = 7; i >= 0; --i){
        if (node->children[i]) {
            write_octree(out, node->children[i]);
        }
       
}
}
void QUANG::write_octree_to_file(octree *root, const char *filename)
{
    std::ofstream out(filename, std::ios::binary);
    if (!out.is_open()) {
        throw std::runtime_error("Could not open file for writing");
    }
    // đếm số nút lá bằng DFS
    out.write((char *)(&root->count), sizeof(root->count));
    out.write((char *)(&root->min_bound), sizeof(QUANG::point));
    out.write((char *)(&root->max_bound), sizeof(QUANG::point));

    write_octree(out, root);
    out.close();
}
QUANG::octree *QUANG::read_octree(std::ifstream &in, const QUANG::point &min_bound, const QUANG::point &max_bound)
{
    octree *node = new octree(min_bound, max_bound);

    QUANG::point center = (min_bound + max_bound) * 0.5f;

    unsigned char child_info;
    in.read((char *)(&child_info), sizeof(unsigned char));
    if (!child_info)
        in.read((char *)&node->attribute, sizeof(QUANG::color));
    for (int octant = 7; octant >=0; --octant)
    {
        if (child_info & (1 << octant))
        {
            QUANG::point new_min = min_bound;
            QUANG::point new_max = center;

            if (octant & 4)
                new_min.x = center.x, new_max.x = max_bound.x;
            if (octant & 2)
                new_min.y = center.y, new_max.y = max_bound.y;
            if (octant & 1)
                new_min.z = center.z, new_max.z = max_bound.z;
            node->children[octant] = read_octree(in, new_min, new_max);
        }
    }
    return node;
}
QUANG::octree *QUANG::load_octree_from_file(const char *filename)
{
    std::ifstream in(filename, std::ios::binary);
    if (!in.is_open())
    {
        throw std::runtime_error("Could not open file for reading");
    }

    QUANG::point min_bound, max_bound;
    int count = 0;
    in.read((char *)(&count), sizeof(count));
    in.read((char *)(&min_bound), sizeof(QUANG::point));
    in.read((char *)(&max_bound), sizeof(QUANG::point));

    octree *root = read_octree(in, min_bound, max_bound);
    in.close();
    root->count = count;
    return root;
}
// // return number of tree
// int QUANG::octree_diff(octree **diff, int* position, octree *ref, octree *pred)
// {
    
// }
