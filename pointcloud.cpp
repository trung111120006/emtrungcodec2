#include "pointcloud.h"

int QUANG::count_verts_ply(char *filename)
{
    miniply::PLYReader reader(filename);
    if (!reader.valid())
    {
        return -1;
    }
    uint32_t elemIndex = reader.find_element("vertex");
    if (elemIndex == miniply::kInvalidIndex)
        return 0;
    return reader.get_element(elemIndex)->count;
}

bool QUANG::load_point_cloud_ply(char *filename, float *pos, unsigned char *color)
{
    miniply::PLYReader reader(filename);
    if (!reader.valid())
    {
        return 0;
    }

    uint32_t propIdxs[3];
    bool gotVerts = false;

    while (reader.has_element() && !gotVerts)
    {
        if (reader.element_is(miniply::kPLYVertexElement))
        {
            if (!reader.load_element() || !reader.find_pos(propIdxs))
            {
                break;
            }

            reader.extract_properties(propIdxs, 3, miniply::PLYPropertyType::Float, pos);
            if (reader.find_color(propIdxs))
            {
                reader.extract_properties(propIdxs, 3, miniply::PLYPropertyType::UChar, color);
            }
            gotVerts = true;
        }
        reader.next_element();
    }

    return 1;
}


void QUANG::PointCloud::deallocate()
{
    if (vertices)
        delete[] vertices;
    if (rgb)
        delete[] rgb;
}

void QUANG::PointCloud::read(const char *path, bool isRGBA)
{
    this->size = count_verts_ply((char *)path);
    this->vertices = new float[this->size * 3];
    this->rgb = new unsigned char[this->size * 3];

    bool chk = load_point_cloud_ply((char *)path, this->vertices, this->rgb);
    if (!chk)
    {
        printf("Load failed.\n");
        return;
    }

}

void QUANG::PointCloud::init(const char *path, bool isRGBA)
{
    deallocate();
    read(path, isRGBA);
}

void QUANG::write_pc_to_ply(QUANG::PointCloud *pc, const char *outpath)
{
    std::ofstream outFile(outpath, std::ios::binary);
    if (!outFile.is_open())
    {
        std::cerr << "Failed to open file: " << outpath << std::endl;
        return;
    }

    // Write the PLY header
    outFile << "ply\n";
    outFile << "format binary_little_endian 1.0\n";
    outFile << "element vertex " << pc->size << "\n";
    outFile << "property float x\n";
    outFile << "property float y\n";
    outFile << "property float z\n";
    outFile << "property uchar red\n";
    outFile << "property uchar green\n";
    outFile << "property uchar blue\n";
    outFile << "end_header\n";

    // Write the vertex data
    for (int i = 0; i < pc->size; ++i)
    {
        // Write the coordinates (x, y, z)
        outFile.write(reinterpret_cast<const char *>(&pc->vertices[3 * i]), sizeof(float) * 3);

        // Write the color (r, g, b)
        outFile.write(reinterpret_cast<const char *>(&pc->rgb[3 * i]), sizeof(unsigned char) * 3);
    }

    outFile.close();
    if (!outFile)
    {
        std::cerr << "Error writing to file: " << outpath << std::endl;
    }
}