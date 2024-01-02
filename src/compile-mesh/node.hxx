#ifndef COMPILE_MESH_NODE_HXX
#define COMPILE_MESH_NODE_HXX

#include <cstdint>
#include <memory>
#include <vector>

#include "data.hxx"

class Node;

class Node {
public:
    std::array<std::array<float, 3>, 2> bounding_box;
    std::weak_ptr<Node> parent_node;
    uint32_t depth, max_depth, node_count;
public:
    static std::shared_ptr<Node> compile(const MeshData &data, const std::vector<uint32_t> &selection, std::shared_ptr<Node> parent = nullptr);
public:
    virtual ~Node() = default;
};

class BranchNode : public virtual Node {
public:
    std::vector<std::shared_ptr<Node>> child_nodes;
public:
    virtual ~BranchNode() = default;
};

class LeafNode : public virtual Node {
public:
    std::vector<uint32_t> triangle_index;
public:
    virtual ~LeafNode() = default;
};

#endif /* COMPILE_MESH_NODE_HXX */
