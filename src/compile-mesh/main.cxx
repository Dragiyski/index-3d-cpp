#include <cassert>
#include <iostream>
#include <numeric>
#include <set>

#include "data.hxx"
#include "node.hxx"
#include "float.hxx"

// #ifdef CM_ENABLE_PROGRESS
#include "progress.hxx"
// #endif

void print_bounding_box(const std::array<std::array<float, 3>, 2> &bounding_box)
{
    std::cerr << "[["
              << bounding_box[0][0] << ", "
              << bounding_box[0][1] << ", "
              << bounding_box[0][2]
              << "], ["
              << bounding_box[1][0] << ", "
              << bounding_box[1][1] << ", "
              << bounding_box[1][2]
              << "]]";
}

void print_tree(std::shared_ptr<Node> node)
{
    if (std::dynamic_pointer_cast<BranchNode>(node) != nullptr)
    {
        auto branch_node = std::dynamic_pointer_cast<BranchNode>(node);
        for (decltype(branch_node->depth) i = 0; i < branch_node->depth; ++i)
        {
            std::cerr << "  ";
        }
        std::cerr << "BranchNode: ";
        std::cerr << "nodes = " << branch_node->node_count << "; ";
        std::cerr << "bounding_box = ";
        print_bounding_box(branch_node->bounding_box);
        std::cerr << std::endl;
        for (const auto &child : branch_node->child_nodes)
        {
            print_tree(child);
        }
    }
    else if (std::dynamic_pointer_cast<LeafNode>(node) != nullptr)
    {
        auto leaf_node = std::dynamic_pointer_cast<LeafNode>(node);
        for (decltype(leaf_node->depth) i = 0; i < leaf_node->depth; ++i)
        {
            std::cerr << "  ";
        }
        std::cerr << "LeafNode: ";
        std::cerr << "triangles = [";
        std::cerr << leaf_node->triangle_index[0];
        for (std::size_t i = 1; i < leaf_node->triangle_index.size(); ++i)
        {
            std::cerr << ", " << leaf_node->triangle_index[i];
        }
        std::cerr << "]"
                  << "; ";
        std::cerr << "bounding_box = ";
        print_bounding_box(leaf_node->bounding_box);
        std::cerr << std::endl;
    }
}

void populate_float_set(std::set<float, float_compare_less<float>> &output, const MeshData &input, std::shared_ptr<Node> node)
{
    // #ifdef CM_ENABLE_PROGRESS
    Progress::get().task_notify();
    // #endif
    for (const auto &minmax : node->bounding_box)
    {
        for (const auto &value : minmax)
        {
            output.insert(value);
        }
    }
    if (std::dynamic_pointer_cast<LeafNode>(node) != nullptr)
    {
        auto leaf_node = std::dynamic_pointer_cast<LeafNode>(node);
        for (const auto &index : leaf_node->triangle_index)
        {
            for (std::size_t i = 0; i < 3; ++i)
            {
                for (const auto &value : input.vertices[input.triangles[index][i][0]])
                {
                    output.insert(value);
                }
                for (const auto &value : input.normals[input.triangles[index][i][1]])
                {
                    output.insert(value);
                }
                for (const auto &value : input.normals[input.triangles[index][i][2]])
                {
                    output.insert(value);
                }
            }
        }
    }
    else if (std::dynamic_pointer_cast<BranchNode>(node) != nullptr)
    {
        auto branch_node = std::dynamic_pointer_cast<BranchNode>(node);
        for (const auto &child : branch_node->child_nodes)
        {
            populate_float_set(output, input, child);
        }
    }
}

void populate_vector_set(std::set<std::array<uint32_t, 2>, array_compare_less<uint32_t, 2>> &vec2, std::set<std::array<uint32_t, 3>, array_compare_less<uint32_t, 3>> &vec3, const std::vector<float> &float_list, const MeshData &input, std::shared_ptr<Node> node)
{
    // #ifdef CM_ENABLE_PROGRESS
    Progress::get().task_notify();
    // #endif
    for (const auto &source_vector : node->bounding_box)
    {
        std::array<uint32_t, 3> index_vector;
        for (std::size_t i = 0; i < 3; ++i)
        {
            auto location = std::lower_bound(float_list.begin(), float_list.end(), source_vector[i]);
            assert(location != float_list.end());
            index_vector[i] = std::distance(float_list.begin(), location);
        }
        vec3.insert(index_vector);
    }
    if (std::dynamic_pointer_cast<LeafNode>(node) != nullptr)
    {
        auto leaf_node = std::dynamic_pointer_cast<LeafNode>(node);
        for (const auto &index : leaf_node->triangle_index)
        {
            for (std::size_t i = 0; i < 3; ++i)
            {
                {
                    std::array<uint32_t, 3> index_vector;
                    for (std::size_t d = 0; d < 3; ++d)
                    {
                        auto location = std::lower_bound(float_list.begin(), float_list.end(), input.vertices[input.triangles[index][i][0]][d]);
                        assert(location != float_list.end());
                        index_vector[d] = std::distance(float_list.begin(), location);
                    }
                    vec3.insert(index_vector);
                }
                {
                    std::array<uint32_t, 3> index_vector;
                    for (std::size_t d = 0; d < 3; ++d)
                    {
                        auto location = std::lower_bound(float_list.begin(), float_list.end(), input.normals[input.triangles[index][i][1]][d]);
                        assert(location != float_list.end());
                        index_vector[d] = std::distance(float_list.begin(), location);
                    }
                    vec3.insert(index_vector);
                }
                {
                    std::array<uint32_t, 2> index_vector;
                    for (std::size_t d = 0; d < 2; ++d)
                    {
                        auto location = std::lower_bound(float_list.begin(), float_list.end(), input.normals[input.triangles[index][i][2]][d]);
                        assert(location != float_list.end());
                        index_vector[d] = std::distance(float_list.begin(), location);
                    }
                    vec2.insert(index_vector);
                }
            }
        }
    }
    else if (std::dynamic_pointer_cast<BranchNode>(node) != nullptr)
    {
        auto branch_node = std::dynamic_pointer_cast<BranchNode>(node);
        for (const auto &child : branch_node->child_nodes)
        {
            populate_vector_set(vec2, vec3, float_list, input, child);
        }
    }
}

template <std::size_t N>
uint32_t find_vec_index(const std::vector<float> float_list, const std::vector<std::array<uint32_t, N>> &vec_list, const std::array<float, N> &target)
{
    std::array<uint32_t, N> index;
    for (std::size_t i = 0; i < N; ++i)
    {
        auto location = std::lower_bound(float_list.begin(), float_list.end(), target[i]);
        assert(location != float_list.end());
        index[i] = std::distance(float_list.begin(), location);
    }
    {
        auto location = std::lower_bound(vec_list.begin(), vec_list.end(), index, array_compare_less<uint32_t, N>{});
        assert(location != vec_list.end());
        return std::distance(vec_list.begin(), location);
    }
}

uint32_t populate_triangle_data(NodeData &output, const MeshData &input, uint32_t triangle_index)
{
    uint32_t node_index = output.node_list.size();
    uint32_t uint_index = output.uint32_list.size();

    output.node_list.push_back(std::array<uint32_t, 4>{uint_index, 0, 0, 0});
    output.uint32_list.push_back(33);
    output.uint32_list.push_back(0b101);
    for (std::size_t k = 0; k < 3; ++k)
    {
        output.uint32_list.push_back(find_vec_index(output.float_list, output.vec3_list, input.vertices[input.triangles[triangle_index][k][0]]));
        output.uint32_list.push_back(find_vec_index(output.float_list, output.vec3_list, input.normals[input.triangles[triangle_index][k][1]]));
        output.uint32_list.push_back(find_vec_index(output.float_list, output.vec2_list, input.texcoords[input.triangles[triangle_index][k][2]]));
    }

    return node_index;
}

uint32_t populate_node_data(NodeData &output, const MeshData &input, std::shared_ptr<Node> node)
{
    // #ifdef CM_ENABLE_PROGRESS
    Progress::get().task_notify();
    // #endif

    uint32_t node_index = output.node_list.size();
    uint32_t uint_index = output.uint32_list.size();
    // Node contains <uint_index, parent, prev_sibling, next_sibling>.
    // The type is the first uint in the uint_list at uint_index
    output.node_list.push_back(std::array<uint32_t, 4>{uint_index, 0, 0, 0});
    output.uint32_list.push_back(88);
    output.uint32_list.push_back(find_vec_index(output.float_list, output.vec3_list, node->bounding_box[0]));
    output.uint32_list.push_back(find_vec_index(output.float_list, output.vec3_list, node->bounding_box[1]));

    if (std::dynamic_pointer_cast<LeafNode>(node) != nullptr)
    {
        auto leaf_node = std::dynamic_pointer_cast<LeafNode>(node);

        output.uint32_list.push_back(leaf_node->triangle_index.size());

        uint32_t children_index = output.uint32_list.size();

        for (std::size_t i = 0; i < leaf_node->triangle_index.size(); ++i)
        {
            output.uint32_list.push_back(0);
        }

        for (std::size_t i = 0; i < leaf_node->triangle_index.size(); ++i)
        {
            auto child_index = populate_triangle_data(output, input, leaf_node->triangle_index[i]);
            output.uint32_list[children_index + i] = child_index;
            output.node_list[child_index][1] = node_index;
        }
        for (std::size_t i = 1; i < leaf_node->triangle_index.size(); ++i)
        {
            output.node_list[output.uint32_list[children_index + i]][2] = output.uint32_list[children_index + i - 1];
        }
        for (std::size_t i = 0; i < leaf_node->triangle_index.size() - 1; ++i)
        {
            output.node_list[output.uint32_list[children_index + i]][3] = output.uint32_list[children_index + i + 1];
        }
    }
    else if (std::dynamic_pointer_cast<BranchNode>(node) != nullptr)
    {
        auto branch_node = std::dynamic_pointer_cast<BranchNode>(node);

        output.uint32_list.push_back(branch_node->child_nodes.size());

        uint32_t children_index = output.uint32_list.size();

        for (std::size_t i = 0; i < branch_node->child_nodes.size(); ++i)
        {
            output.uint32_list.push_back(0);
        }

        for (std::size_t i = 0; i < branch_node->child_nodes.size(); ++i)
        {
            auto child_index = populate_node_data(output, input, branch_node->child_nodes[i]);
            output.uint32_list[children_index + i] = child_index;
            output.node_list[child_index][1] = node_index;
        }
        for (std::size_t i = 1; i < branch_node->child_nodes.size(); ++i)
        {
            output.node_list[output.uint32_list[children_index + i]][2] = output.uint32_list[children_index + i - 1];
        }
        for (std::size_t i = 0; i < branch_node->child_nodes.size() - 1; ++i)
        {
            output.node_list[output.uint32_list[children_index + i]][3] = output.uint32_list[children_index + i + 1];
        }
    }
    else
    {
        abort();
    }

    return node_index;
}

int main(int argc, char *argv[])
{
    MeshData data;
    BufferSize size_vertex, size_normal, size_texcoord, size_triangle;

    // #ifdef CM_ENABLE_PROGRESS
    std::cerr << "Acquiring data from the input stream..." << std::endl;
    // #endif

    std::cin.read(size_vertex.bytes, sizeof(BufferSize));
    std::cin.read(size_normal.bytes, sizeof(BufferSize));
    std::cin.read(size_texcoord.bytes, sizeof(BufferSize));
    std::cin.read(size_triangle.bytes, sizeof(BufferSize));

    data.vertices.resize(size_vertex.value);
    data.normals.resize(size_normal.value);
    data.texcoords.resize(size_texcoord.value);
    data.triangles.resize(size_triangle.value);

    std::cin.read(reinterpret_cast<char *>(data.vertices.data()), size_vertex.value * sizeof(decltype(data.vertices)::value_type));
    std::cin.read(reinterpret_cast<char *>(data.normals.data()), size_normal.value * sizeof(decltype(data.normals)::value_type));
    std::cin.read(reinterpret_cast<char *>(data.texcoords.data()), size_texcoord.value * sizeof(decltype(data.texcoords)::value_type));
    std::cin.read(reinterpret_cast<char *>(data.triangles.data()), size_triangle.value * sizeof(decltype(data.triangles)::value_type));

    // #ifdef CM_ENABLE_PROGRESS
    Progress::get().update("Compile nodes");
    // #endif

    std::vector<uint32_t> selection(static_cast<std::size_t>(size_triangle.value));
    std::iota(selection.begin(), selection.end(), 0);
    auto root_node = Node::compile(data, selection);

    // #ifdef CM_ENABLE_PROGRESS
    Progress::get().done();
    // #endif

    NodeData node_data;

    // print_tree(root_node);
    {
        // #ifdef CM_ENABLE_PROGRESS
        Progress::get().update("Generating float list", root_node->node_count);
        // #endif
        std::set<float, float_compare_less<float>> float_set;
        populate_float_set(float_set, data, root_node);
        // #ifdef CM_ENABLE_PROGRESS
        Progress::get().done();
        // #endif
        node_data.float_list.reserve(float_set.size() + 1);
        node_data.float_list.push_back(std::numeric_limits<float>::quiet_NaN());
        for (const auto &value : float_set)
        {
            node_data.float_list.push_back(value);
        }
    }
    {
        // #ifdef CM_ENABLE_PROGRESS
        Progress::get().update("Generating vector list", root_node->node_count);
        // #endif
        std::set<std::array<uint32_t, 2>, array_compare_less<uint32_t, 2>> vec2_set;
        std::set<std::array<uint32_t, 3>, array_compare_less<uint32_t, 3>> vec3_set;
        populate_vector_set(vec2_set, vec3_set, node_data.float_list, data, root_node);
        // #ifdef CM_ENABLE_PROGRESS
        Progress::get().done();
        // #endif
        node_data.vec2_list.reserve(vec2_set.size() + 1);
        node_data.vec2_list.push_back({0, 0});
        for (const auto &value : vec2_set)
        {
            node_data.vec2_list.push_back(value);
        }

        node_data.vec3_list.reserve(vec3_set.size() + 1);
        node_data.vec3_list.push_back({0, 0, 0});
        for (const auto &value : vec3_set)
        {
            node_data.vec3_list.push_back(value);
        }

        // #ifdef CM_ENABLE_PROGRESS
        Progress::get().update("Populating node data", root_node->node_count);
        // #endif
        auto root_node_index = populate_node_data(node_data, data, root_node);
        // #ifdef CM_ENABLE_PROGRESS
        Progress::get().done();
        // #endif

        {
            std::cerr << std::endl
                      << "Stats: " << std::endl;
            std::cerr << "  - float: "
                      << node_data.float_list.size()
                      << " * "
                      << sizeof(decltype(node_data.float_list)::value_type) << "B"
                      << " = "
                      << node_data.float_list.size() * sizeof(decltype(node_data.float_list)::value_type) << "B"
                      << std::endl;
            std::cerr << "  - vec2: "
                      << node_data.vec2_list.size()
                      << " * "
                      << sizeof(decltype(node_data.vec2_list)::value_type) << "B"
                      << " = "
                      << node_data.vec2_list.size() * sizeof(decltype(node_data.vec2_list)::value_type) << "B"
                      << std::endl;
            std::cerr << "  - vec3: "
                      << node_data.vec3_list.size()
                      << " * "
                      << sizeof(decltype(node_data.vec3_list)::value_type) << "B"
                      << " = "
                      << node_data.vec3_list.size() * sizeof(decltype(node_data.vec3_list)::value_type) << "B"
                      << std::endl;
            std::cerr << "  - uint32: "
                      << node_data.uint32_list.size()
                      << " * "
                      << sizeof(decltype(node_data.uint32_list)::value_type) << "B"
                      << " = "
                      << node_data.uint32_list.size() * sizeof(decltype(node_data.uint32_list)::value_type) << "B"
                      << std::endl;
            std::cerr << "  - node: "
                      << node_data.node_list.size()
                      << " * "
                      << sizeof(decltype(node_data.node_list)::value_type) << "B"
                      << " = "
                      << node_data.node_list.size() * sizeof(decltype(node_data.node_list)::value_type) << "B"
                      << std::endl;
        }

        {
            BufferSize float_size, vec2_size, vec3_size, zero_size, uint32_size, node_size, root_node_value;
            zero_size.value = 0;
            float_size.value = node_data.float_list.size();
            vec2_size.value = node_data.vec2_list.size();
            vec3_size.value = node_data.vec3_list.size();
            uint32_size.value = node_data.uint32_list.size();
            node_size.value = node_data.node_list.size();
            root_node_value.value = root_node_index;
            std::cout.write(float_size.bytes, sizeof(BufferSize));
            std::cout.write(vec2_size.bytes, sizeof(BufferSize));
            std::cout.write(vec3_size.bytes, sizeof(BufferSize));
            std::cout.write(zero_size.bytes, sizeof(BufferSize));
            std::cout.write(uint32_size.bytes, sizeof(BufferSize));
            std::cout.write(node_size.bytes, sizeof(BufferSize));
            std::cout.write(root_node_value.bytes, sizeof(BufferSize));
            std::cout.write(zero_size.bytes, sizeof(BufferSize));
            std::cout.write(zero_size.bytes, sizeof(BufferSize));
            std::cout.write(reinterpret_cast<char *>(node_data.float_list.data()), node_data.float_list.size() * sizeof(decltype(node_data.float_list)::value_type));
            std::cout.write(reinterpret_cast<char *>(node_data.vec2_list.data()), node_data.vec2_list.size() * sizeof(decltype(node_data.vec2_list)::value_type));
            std::cout.write(reinterpret_cast<char *>(node_data.vec3_list.data()), node_data.vec3_list.size() * sizeof(decltype(node_data.vec3_list)::value_type));
            std::cout.write(reinterpret_cast<char *>(node_data.uint32_list.data()), node_data.uint32_list.size() * sizeof(decltype(node_data.uint32_list)::value_type));
            std::cout.write(reinterpret_cast<char *>(node_data.node_list.data()), node_data.node_list.size() * sizeof(decltype(node_data.node_list)::value_type));
        }

        // Object data consists of only pointers to integer data (which contains pointers to vector and float lists).
        // Inteher data is a variable size data of instructions specifying how to handle the node. There might be a sequence of instructions, similar to assembly representation of a function.
        // In that regard, object data, vec2/3/4 data, float_list, etc. is considered memory to those instructions.
        // Raytracing must produce result color + additional data (how much) to be handled by subsequent phases (how to detect if any?)
        // Object data format:
        // Node type;
        // PTR to integer instruction/data list;
        // PTR to tree data (in integer), if any;
        // PTR to dynamic data for writing, if any;

        // Node data is 4-component uint32_t, we need:
        // Type/ID: a value to determine how to process the node;
        // Flags?
        // PTR: integer data, containing variable length data for the current object;
        // PTR: tree data within integer data;

        // Type/ID determines how the node is processed and what data is required...
        // BoundingBox Node: the data contains 2 uint32 indices within vec3`
    }

    return 0;
}