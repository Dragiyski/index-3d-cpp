#ifndef COMPILE_MESH_DATA_HXX
#define COMPILE_MESH_DATA_HXX

#include <array>
#include <cstdint>
#include <vector>

class MeshData {
public:
    std::vector<std::array<float, 3>> vertices, normals;
    std::vector<std::array<float, 2>> texcoords;
    std::vector<std::array<std::array<uint32_t, 3>, 3>> triangles;
};

class NodeData {
public:
    uint32_t root_node;
    std::vector<float> float_list;
    std::vector<uint32_t> uint32_list;
    std::vector<std::array<uint32_t, 2>> vec2_list;
    std::vector<std::array<uint32_t, 3>> vec3_list;
    std::vector<std::array<uint32_t, 4>> node_list;
};

typedef union {
    char bytes[4];
    std::uint32_t value;
} BufferSize;

template <typename T, std::size_t N>
struct array_compare_less {
    bool operator()(const std::array<T, N> &a, const std::array<T, N> &b) const {
        for (std::size_t i = 0; i < N; ++i) {
            if (a[i] > b[i]) {
                return false;
            } else if (a[i] < b[i]) {
                return true;
            }
        }
        return false;
    }
};

#endif /* COMPILE_MESH_DATA_HXX */