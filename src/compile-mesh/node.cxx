#include "node.hxx"
#include "progress.hxx"
#include <cassert>
#include <limits>
#include <numeric>

std::shared_ptr<Node> Node::compile(const MeshData &data, const std::vector<uint32_t> &selection, std::shared_ptr<Node> parent) {
    // #ifdef CM_ENABLE_PROGRESS
    Progress::get().task_notify();
    // #endif
    static const auto inf = std::numeric_limits<float>::infinity();
    decltype(std::declval<Node>().bounding_box) bounding_box{
        +inf, +inf, +inf, -inf, -inf, -inf
    };
    for (const auto &index : selection) {
        auto triangle = data.triangles[index];
        for (const auto &indices : triangle) {
            auto vertex = data.vertices[indices[0]];
            for (std::size_t dim = 0; dim < 3; ++dim) {
                auto value = vertex[dim];
                bounding_box[0][dim] = std::min(bounding_box[0][dim], value);
                bounding_box[1][dim] = std::max(bounding_box[1][dim], value);
            }
        }
    }
    if (selection.size() <= 6) {
        auto node = std::make_shared<LeafNode>();
        node->bounding_box = bounding_box;
        node->depth = parent ? parent->depth + 1 : 0;
        node->max_depth = 1;
        node->parent_node = parent;
        node->triangle_index = selection;
        node->node_count = 1;
        return node;
    }
    std::vector<std::vector<std::vector<uint32_t>>> distribution_list;
    for (std::size_t dim = 0; dim < 3; ++dim) {
        std::vector<std::pair<uint32_t, float>> min_vertex_list, max_vertex_list, affinity_list;
        for (const auto &index : selection) {
            std::array vertex_value{
                data.vertices[data.triangles[index][0][0]][dim],
                data.vertices[data.triangles[index][1][0]][dim],
                data.vertices[data.triangles[index][2][0]][dim]
            };
            auto min_value = *std::min_element(vertex_value.begin(), vertex_value.end());
            auto max_value = *std::max_element(vertex_value.begin(), vertex_value.end());
            min_vertex_list.push_back(std::make_pair(index, min_value));
            max_vertex_list.push_back(std::make_pair(index, max_value));
            affinity_list.push_back(std::make_pair(index, (min_value - bounding_box[0][dim]) - (bounding_box[1][dim] - max_value)));
        }
        std::sort(affinity_list.begin(), affinity_list.end(), [](decltype(affinity_list)::value_type a, decltype(affinity_list)::value_type b) {
            return a.second < b.second;
        });
        std::vector<std::pair<uint32_t, float>> min_group, max_group;
        min_group.reserve(affinity_list.size() / 2);
        max_group.reserve(affinity_list.size() - affinity_list.size() / 2);
        for (auto index = 0; index < affinity_list.size() / 2; ++index) {
            auto location = std::lower_bound(max_vertex_list.begin(), max_vertex_list.end(), affinity_list[index].first, [](decltype(max_vertex_list)::value_type a, decltype(affinity_list[index].first) b) {
                return a.first < b;
            });
            assert(location != max_vertex_list.end());
            min_group.push_back(*location);
        }
        for (auto index = min_group.size(); index < affinity_list.size(); ++index) {
            auto location = std::lower_bound(min_vertex_list.begin(), min_vertex_list.end(), affinity_list[index].first, [](decltype(min_vertex_list)::value_type a, decltype(affinity_list[index].first) b) {
                return a.first < b;
            });
            assert(location != min_vertex_list.end());
            max_group.push_back(*location);
        }
        std::sort(min_group.begin(), min_group.end(), [](decltype(min_group)::value_type a, decltype(min_group)::value_type b) {
            return a.second < b.second;
        });
        std::sort(max_group.begin(), max_group.end(), [](decltype(max_group)::value_type a, decltype(max_group)::value_type b) {
            return a.second > b.second;
        });
        std::vector<uint32_t> mix_group;
        for (auto min_isect = max_group.back().second, max_isect = min_group.back().second; min_isect < max_isect && mix_group.size() < selection.size() / 3;) {
            if (min_group.size() > max_group.size()) {
                mix_group.push_back(min_group.back().first);
                min_group.pop_back();
            } else {
                mix_group.push_back(max_group.back().first);
                max_group.pop_back();
            }
            min_isect = max_group.back().second;
            max_isect = min_group.back().second;
        }
        decltype(distribution_list)::value_type distribution;
        if (min_group.size() > 0) {
            decltype(distribution)::value_type group;
            group.reserve(min_group.size());
            for (const auto &value : min_group) {
                group.push_back(value.first);
            }
            distribution.push_back(group);
        }
        if (max_group.size() > 0) {
            decltype(distribution)::value_type group;
            group.reserve(max_group.size());
            for (const auto &value : max_group) {
                group.push_back(value.first);
            }
            distribution.push_back(group);
        }
        if (mix_group.size() > 0) {
            distribution.push_back(mix_group);
        }
        if (distribution.size() >= 2) {
            distribution_list.push_back(distribution);
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
        node->bounding_box = bounding_box;
        node->depth = parent ? parent->depth + 1 : 0;
        node->parent_node = parent;
        node->max_depth = 1;
        node->node_count = 1;
        for (auto &group : distribution) {
            std::sort(group.begin(), group.end());
            auto sub_node = compile(data, group, node);
            if (!sub_node) {
                goto next_dim;
            }
            node->child_nodes.push_back(sub_node);
        }
        if (node->child_nodes.size() > 1) {
            for (const auto &child : node->child_nodes) {
                auto new_depth = 1 + child->max_depth;
                if (new_depth > node->max_depth) {
                    node->max_depth = new_depth;
                }
                node->node_count += child->node_count;
            }
            return node;
        }
        next_dim:;
    }
    return nullptr;
}
