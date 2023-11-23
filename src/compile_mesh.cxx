#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <numeric>
#include <limits>
#include <ranges>
#include <set>
#include <stdexcept>
#include <vector>

typedef union {
    char bytes[4];
    std::uint32_t value;
} BufferSize;

class Node;

class Node {
public:
    bool is_container;
    std::array<std::array<float, 3>, 2> bounding_box;
    std::weak_ptr<Node> parent_node;
    std::size_t depth;
};

class BranchNode : public Node {
public:
    std::vector<std::shared_ptr<Node>> child_nodes;
};

class LeafNode : public Node {
public:
    std::vector<uint32_t> triangle_index;
};

class MeshData {
public:
    std::vector<float> float_list;
    std::vector<std::array<uint32_t, 3>> vertices, normals;
    std::vector<std::array<uint32_t, 2>> texcoords;
    std::vector<std::array<std::array<uint32_t, 3>, 3>> triangles;
};

template <typename T>
struct float_is_equal {
    bool operator()(T a, T b) const {
        static const auto epsilon = std::numeric_limits<T>::epsilon();
        static const auto min_value = std::numeric_limits<T>::min();
        static const auto max_value = std::numeric_limits<T>::max();
        // Nothing is equal to NaN (not even NaN)
        if (std::isnan(a) || std::isnan(b)) {
            return false;
        }
        // If both are exactly equal, they are equal, no further checks.
        if (a == b) {
            return true;
        }
        auto difference = std::abs(a - b);
        auto finite_amplitude = std::min(std::abs(a) + std::abs(b), max_value);
        auto min_difference = std::max(min_value, finite_amplitude * epsilon);
        return difference < min_difference;
    }
};

template <typename T>
struct float_compare_less {
    bool operator()(T a, T b) const {
        static const auto is_equal_function = float_is_equal<T>();
        if (!is_equal_function(a, b)) {
            return a < b;
        }
        return false;
    }
};

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

template<std::size_t Dims>
void append_float_list(std::set<float, float_compare_less<float>> &float_list, const std::vector<std::array<float, Dims>> &vector_list) {
    static const auto max_index = std::numeric_limits<uint32_t>::max();
    for (const auto &vector : vector_list) {
        for (const auto &value : vector) {
            float_list.insert(value);
            // No checks requires, there are less than 2^32 unique floating-point values, excluding NaN, Infinities and other special numbers.
        }
    }
}

template<std::size_t Dims>
void create_index_list(std::vector<std::array<uint32_t, Dims>> &index_list, const std::vector<std::array<float, Dims>> &value_list, const std::set<float, float_compare_less<float>> &float_list) {
    for (std::size_t index = 0; index < value_list.size(); ++index) {
        for (std::size_t dim = 0; dim < Dims; ++dim) {
            auto value = value_list[index][dim];
            auto location = float_list.find(value);
            auto target_index = std::distance(float_list.begin(), location);
            index_list[index][dim] = target_index;
        }
    }
}

std::shared_ptr<Node> compile_node(const MeshData &data, const std::vector<uint32_t> &selection, std::shared_ptr<Node> parent = nullptr) {
    static const auto inf = std::numeric_limits<float>::infinity();
    decltype(std::declval<Node>().bounding_box) bounding_box{
        +inf, +inf, +inf, -inf, -inf, -inf
    };
    for (const auto &index : selection) {
        auto triangle = data.triangles[index];
        for (const auto &indices : triangle) {
            auto vertex = data.vertices[indices[0]];
            for (std::size_t dim = 0; dim < 3; ++dim) {
                auto value = data.float_list[vertex[dim]];
                bounding_box[0][dim] = std::min(bounding_box[0][dim], value);
                bounding_box[1][dim] = std::max(bounding_box[1][dim], value);
            }
        }
    }
    if (selection.size() <= 6) {
        auto node = std::make_shared<LeafNode>();
        node->is_container = false;
        node->bounding_box = bounding_box;
        node->depth = parent ? parent->depth + 1 : 0;
        node->parent_node = parent;
        node->triangle_index = selection;
        return node;
    }
    std::vector<std::vector<std::vector<uint32_t>>> distribution_list;
    for (std::size_t dim = 0; dim < 3; ++dim) {
        std::map<uint32_t, float> min_vertex_value, max_vertex_value;
        auto sort_index_by_min_vertex = [&min_vertex_value](uint32_t a, uint32_t b) {
            return min_vertex_value[a] < min_vertex_value[b];
        };
        auto sort_index_by_max_vertex = [&max_vertex_value](uint32_t a, uint32_t b) {
            return max_vertex_value[a] < max_vertex_value[b];
        };
        auto compute_affinity = [&min_vertex_value, &max_vertex_value, &bounding_box, &dim](uint32_t index) -> auto {
            return (min_vertex_value[index] - bounding_box[0][dim]) - (bounding_box[1][dim] - max_vertex_value[index]);
        };
        auto sort_index_by_affinity = [&compute_affinity](uint32_t a, uint32_t b) {
            return compute_affinity(a) < compute_affinity(b);
        };
        std::multiset<uint32_t, decltype(sort_index_by_min_vertex)> min_vertex_index(sort_index_by_min_vertex);
        std::multiset<uint32_t, decltype(sort_index_by_max_vertex)> max_vertex_index(sort_index_by_max_vertex);
        std::multiset<uint32_t, decltype(sort_index_by_affinity)> affinity_index(sort_index_by_affinity);
        for (uint32_t index = 0; index < selection.size(); ++index) {
            std::array vertex_value{
                data.float_list[data.vertices[data.triangles[selection[index]][0][0]][dim]],
                data.float_list[data.vertices[data.triangles[selection[index]][1][0]][dim]],
                data.float_list[data.vertices[data.triangles[selection[index]][2][0]][dim]]
            };
            min_vertex_value[selection[index]] = *std::min_element(vertex_value.begin(), vertex_value.end());
            max_vertex_value[selection[index]] = *std::max_element(vertex_value.begin(), vertex_value.end());
            min_vertex_index.insert(selection[index]);
            max_vertex_index.insert(selection[index]);
            affinity_index.insert(selection[index]);
        }
        std::multiset<uint32_t, decltype(sort_index_by_max_vertex)> min_group(sort_index_by_max_vertex);
        std::multiset<uint32_t, decltype(sort_index_by_min_vertex)> max_group(sort_index_by_min_vertex);
        for (const auto &item : affinity_index | std::ranges::views::take(selection.size() / 2)) {
            min_group.insert(item);
        }
        for (const auto &item : affinity_index | std::ranges::views::drop(selection.size() / 2)) {
            max_group.insert(item);
        }
        std::multiset<uint32_t, decltype(sort_index_by_affinity)> mix_group(sort_index_by_affinity);
        for (auto min_isect = min_vertex_value[*max_group.begin()], max_isect = max_vertex_value[*(--min_group.end())]; min_isect < max_isect && mix_group.size() < selection.size() / 3;) {
            bool is_min_group = false;
            auto min_boundary = --min_group.end();
            auto max_boundary = max_group.begin();
            if (min_group.size() > max_group.size()) {
                is_min_group = true;
            } else {
                is_min_group = false;
            }
            auto value = is_min_group ? *min_boundary : *max_boundary;
            if (is_min_group) {
                min_group.erase(min_boundary);
                // min_cumdiff.pop_back();
            } else {
                max_group.erase(max_boundary);
                // max_cumdiff.pop_back();
            }
            mix_group.insert(value);
            min_isect = min_vertex_value[*max_group.begin()];
            max_isect = max_vertex_value[*(--min_group.end())];
        }
        decltype(distribution_list)::value_type distribution;
        {
            decltype(distribution)::value_type group;
            std::copy(min_group.begin(), min_group.end(), std::inserter(group, group.end()));
            distribution.push_back(group);
        }
        {
            decltype(distribution)::value_type group;
            std::copy(max_group.begin(), max_group.end(), std::inserter(group, group.end()));
            distribution.push_back(group);
        }
        if (mix_group.size() > 0) {
            decltype(distribution)::value_type group;
            std::copy(mix_group.begin(), mix_group.end(), std::inserter(group, group.end()));
            distribution.push_back(group);
        }
        distribution_list.push_back(distribution);

        std::cout
            << "Dimension" << ": " << dim << ", "
            << "Distribution" << ": " << "{" << distribution[0].size() << ", " <<  distribution[1].size() << ", " << distribution[2].size() << "}"
            << std::endl;
        std::cout << "min_group" << ": " << distribution[0].size() << " " << "triangles" << std::endl;
        for (const auto &index : distribution[0]) {
            std::cout
                << "  [" << index << "]" << ": "
                << std::setprecision(6) << std::fixed
                << "min_value=" << min_vertex_value[index] << ", "
                << "max_value=" << max_vertex_value[index] << ", "
                << "affinity=" << compute_affinity(index)
                << std::endl;
        }
        std::cout << "max_group" << ": " << distribution[1].size() << " " << "triangles" << std::endl;
        for (const auto &index : distribution[1]) {
            std::cout
                << "  [" << index << "]" << ": "
                << std::setprecision(6) << std::fixed
                << "min_value=" << min_vertex_value[index] << ", "
                << "max_value=" << max_vertex_value[index] << ", "
                << "affinity=" << compute_affinity(index)
                << std::endl;
        }
        if (distribution.size() > 2) {
            std::cout << "mid_group" << ": " << distribution[2].size() << " " << "triangles" << std::endl;
            for (const auto &index : distribution[2]) {
                std::cout
                    << "  [" << index << "]" << ": "
                    << std::setprecision(6) << std::fixed
                    << "min_value=" << min_vertex_value[index] << ", "
                    << "max_value=" << max_vertex_value[index] << ", "
                    << "affinity=" << compute_affinity(index)
                    << std::endl;
            }
        }
    }
    auto compute_rating = [&distribution_list](uint32_t dim) {
        auto imbalance = std::abs(static_cast<int64_t>(distribution_list[dim][0].size()) - static_cast<int64_t>(distribution_list[dim][1].size()));
        int64_t mid_size = distribution_list[dim].size() > 2 ? distribution_list[dim][2].size() : 0;
        return imbalance + mid_size;
    };
    std::vector<uint32_t> rating_list(distribution_list.size());
    std::iota(rating_list.begin(), rating_list.end(), 0);
    std::sort(rating_list.begin(), rating_list.end(), [&compute_rating](uint32_t a, uint32_t b) {
        return compute_rating(a) < compute_rating(b);
    });
    for (const auto &dim : rating_list) {
        auto distribution = distribution_list[dim];
        auto node = std::make_shared<BranchNode>();
        node->is_container = true;
        node->bounding_box = bounding_box;
        node->depth = parent ? parent->depth + 1 : 0;
        node->parent_node = parent;
        for (const auto &group : distribution) {
            auto sub_node = compile_node(data, group, node);
            if (!sub_node) {
                goto next_dim;
            }
            node->child_nodes.push_back(sub_node);
        }
        if (node->child_nodes.size() > 1) {
            return node;
        }
        next_dim:;
    }
    return nullptr;
}

int main(int argc, char *argv[]) {
    BufferSize vertex_count, normal_count, texcoord_count, triangle_count;
    std::vector<std::array<float, 3>> vertex_buffer, normal_buffer;
    std::vector<std::array<float, 2>> texcoord_buffer;
    std::vector<std::array<std::array<uint32_t, 3>, 3>> triangle_buffer;
    MeshData mesh_data;

    std::cin.read(vertex_count.bytes, 4);
    std::cin.read(normal_count.bytes, 4);
    std::cin.read(texcoord_count.bytes, 4);
    std::cin.read(triangle_count.bytes, 4);

    vertex_buffer.resize(vertex_count.value);
    normal_buffer.resize(normal_count.value);
    texcoord_buffer.resize(texcoord_count.value);
    mesh_data.triangles.resize(triangle_count.value);

    std::cin.read(reinterpret_cast<char *>(vertex_buffer.data()), vertex_count.value * sizeof(decltype(vertex_buffer)::value_type));
    std::cin.read(reinterpret_cast<char *>(normal_buffer.data()), normal_count.value * sizeof(decltype(normal_buffer)::value_type));
    std::cin.read(reinterpret_cast<char *>(texcoord_buffer.data()), texcoord_count.value * sizeof(decltype(texcoord_buffer)::value_type));
    std::cin.read(reinterpret_cast<char *>(mesh_data.triangles.data()), triangle_count.value * sizeof(decltype(mesh_data.triangles)::value_type));

    std::set<float, float_compare_less<float>> float_set;
    append_float_list(float_set, vertex_buffer);
    append_float_list(float_set, normal_buffer);
    append_float_list(float_set, texcoord_buffer);

    std::vector<float> float_list;
    float_list.reserve(float_set.size());
    for(const auto &value : float_set) {
        mesh_data.float_list.push_back(value);
    }

    std::vector<std::array<uint32_t, 3>> vertex_index_buffer, normal_index_buffer;
    std::vector<std::array<uint32_t, 2>> texcoord_index_buffer;

    mesh_data.vertices.resize(vertex_count.value);
    mesh_data.normals.resize(normal_count.value);
    mesh_data.texcoords.resize(texcoord_count.value);
    create_index_list(mesh_data.vertices, vertex_buffer, float_set);
    create_index_list(mesh_data.normals, normal_buffer, float_set);
    create_index_list(mesh_data.texcoords, texcoord_buffer, float_set);

    std::vector<uint32_t> selection_list(static_cast<std::size_t>(triangle_count.value));
    std::iota(selection_list.begin(), selection_list.end(), 0);

    auto node = compile_node(mesh_data, selection_list);

    // TODO Setup std::array<uint32_t, Dims> compare function to create std::set<std::array<uint32_t>>. Collect all triplets in one buffer, texcoord in another.

    return 0;
}