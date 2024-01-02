#include <algorithm>
#include <array>
#include <cassert>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
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
    uint32_t depth;
    uint32_t max_depth;
    uint32_t node_count;
};

std::thread state_thread;
std::string state_prefix;
std::size_t state_total = 0;
std::size_t state_completed = 0;
std::mutex state_mutex;
std::condition_variable state_sync, state_wait_print_sync;
std::chrono::steady_clock::time_point state_last_update;
bool state_should_update = true;
bool state_is_running = false;
bool state_is_exiting = false;
bool state_wait_for_print = false;
bool state_has_atexit = false;

void state_main() {
    using namespace std::chrono_literals;

    std::unique_lock state_lock(state_mutex);
    while (true) {
        if (state_is_exiting) {
            return;
        }
        if (state_is_running) {
            auto now = std::chrono::steady_clock::now();
            if (state_should_update || now - state_last_update >= 1000ms) {
                state_should_update = false;
                state_last_update = now;
                if (state_total > 0) {
                    std::cerr << state_prefix << " " << "(" << state_completed << " / " << state_total << ")" << std::endl;
                } else {
                    std::cerr << state_prefix << " " << "(" << state_completed << ")" << std::endl;
                }
            }
        }
        if (state_wait_for_print) {
            state_wait_for_print = false;
            state_wait_print_sync.notify_all();
        }
        state_sync.wait_for(state_lock, 1000ms);
    }
}

void state_atexit() {
    if (state_thread.joinable()) {
        {
            std::lock_guard state_lock(state_mutex);
            state_is_exiting = true;
            state_sync.notify_all();
        }
        state_thread.join();
    }
}

void state_ensure_thread() {
    if (!state_thread.joinable()) {
        state_thread = std::thread(state_main);
        if (!state_has_atexit) {
            state_has_atexit = true;
            std::atexit(state_atexit);
        }
    }
}

void state_set(const std::string &prefix, uint32_t total = 0) {
    state_ensure_thread();
    std::unique_lock state_lock(state_mutex);
    state_is_running = true;
    state_is_exiting = false;
    state_should_update = true;
    state_prefix = prefix;
    state_total = total;
    state_completed = 0;
    state_wait_for_print = true;
    state_sync.notify_all();
    state_wait_print_sync.wait(state_lock, []() { return state_wait_for_print == false; });
}

void state_pause() {
    std::lock_guard state_lock(state_mutex);
    state_is_running = false;
}

void state_exit() {
    std::lock_guard state_lock(state_mutex);
    state_is_exiting = true;
    state_should_update = true;
    state_sync.notify_all();
}

void state_update(std::size_t completed) {
    std::lock_guard state_lock(state_mutex);
    state_completed = completed;
}

void state_increase(std::size_t difference) {
    std::lock_guard state_lock(state_mutex);
    state_completed += difference;
}

void state_finish() {
    state_ensure_thread();
    std::unique_lock state_lock(state_mutex);
    state_should_update = true;
    state_wait_for_print = true;
    state_sync.notify_all();
    state_wait_print_sync.wait(state_lock, []() { return state_wait_for_print == false; });
}

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
            state_increase(1);
            // No checks requires, there are less than 2^32 unique floating-point values, excluding NaN, Infinities and other special numbers.
        }
    }
}

template<std::size_t Dims>
void create_index_list(std::vector<std::array<uint32_t, Dims>> &index_list, const std::vector<std::array<float, Dims>> &value_list, const std::vector<float> &float_list) {
    static const auto float_compare = float_compare_less<float>();
    for (std::size_t index = 0; index < value_list.size(); ++index) {
        for (std::size_t dim = 0; dim < Dims; ++dim) {
            auto value = value_list[index][dim];
            auto location = std::lower_bound(float_list.begin(), float_list.end(), value, float_compare);
            assert(location != float_list.end());
            auto target_index = std::distance(float_list.begin(), location);
            index_list[index][dim] = target_index;
            state_increase(1);
        }
    }
}

std::shared_ptr<Node> compile_node(const MeshData &data, const std::vector<uint32_t> &selection, std::shared_ptr<Node> parent = nullptr) {
    state_increase(1);
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
                data.float_list[data.vertices[data.triangles[index][0][0]][dim]],
                data.float_list[data.vertices[data.triangles[index][1][0]][dim]],
                data.float_list[data.vertices[data.triangles[index][2][0]][dim]]
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
        node->is_container = true;
        node->bounding_box = bounding_box;
        node->depth = parent ? parent->depth + 1 : 0;
        node->parent_node = parent;
        node->max_depth = 1;
        node->node_count = 1;
        for (const auto &group : distribution) {
            auto sub_node = compile_node(data, group, node);
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

uint32_t insert_node(std::vector<float> &float_data, std::vector<uint32_t> &int_data, std::vector<std::array<uint32_t, 4>> &object_data, std::shared_ptr<Node> node) {
    state_increase(1);
    auto object_base = object_data.size();
    auto int_base = int_data.size();
    auto float_base = float_data.size();
    for (const auto &minmax : node->bounding_box) {
        for (const auto &value : minmax) {
            float_data.push_back(value);
        }
    }
    if (node->is_container) {
        auto container_node = std::reinterpret_pointer_cast<BranchNode>(node);
        object_data.push_back(std::array<uint32_t, 4>{ 65, static_cast<uint32_t>(container_node->child_nodes.size()), static_cast<uint32_t>(int_base), static_cast<uint32_t>(float_base) });
        for (std::size_t i = 0; i < container_node->child_nodes.size(); ++i) {
            int_data.push_back(static_cast<uint32_t>(-1));
        }
        auto child_base = int_base;
        for (std::size_t i = 0; i < container_node->child_nodes.size(); ++i) {
            auto child_node = container_node->child_nodes[i];
            auto child_index = insert_node(float_data, int_data, object_data, child_node);
            int_data[child_base + i] = child_index;
        }
    } else {
        auto leaf_node = std::reinterpret_pointer_cast<LeafNode>(node);
        object_data.push_back(std::array<uint32_t, 4>{ 66, static_cast<uint32_t>(leaf_node->triangle_index.size()), static_cast<uint32_t>(int_base), static_cast<uint32_t>(float_base) });
        for (const auto &index : leaf_node->triangle_index) {
            int_data.push_back(index);
        }
    }
    return object_base;
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

    state_set(
        "Building float list",
        vertex_buffer.size() * std::tuple_size<decltype(vertex_buffer)::value_type>::value +
        normal_buffer.size() * std::tuple_size<decltype(normal_buffer)::value_type>::value +
        texcoord_buffer.size() * std::tuple_size<decltype(texcoord_buffer)::value_type>::value
    );

    std::set<float, float_compare_less<float>> float_set;
    append_float_list(float_set, vertex_buffer);
    append_float_list(float_set, normal_buffer);
    append_float_list(float_set, texcoord_buffer);

    state_finish();

    state_set("Copy float set to float list", float_set.size());

    mesh_data.float_list.reserve(float_set.size());
    for(const auto &value : float_set) {
        mesh_data.float_list.push_back(value);
        state_increase(1);
    }

    state_finish();

    std::vector<std::array<uint32_t, 3>> vertex_index_buffer, normal_index_buffer;
    std::vector<std::array<uint32_t, 2>> texcoord_index_buffer;

    state_set(
        "Building vector list",
        vertex_buffer.size() * std::tuple_size<decltype(vertex_buffer)::value_type>::value +
        normal_buffer.size() * std::tuple_size<decltype(normal_buffer)::value_type>::value +
        texcoord_buffer.size() * std::tuple_size<decltype(texcoord_buffer)::value_type>::value
    );

    mesh_data.vertices.resize(vertex_count.value);
    mesh_data.normals.resize(normal_count.value);
    mesh_data.texcoords.resize(texcoord_count.value);
    create_index_list(mesh_data.vertices, vertex_buffer, mesh_data.float_list);
    create_index_list(mesh_data.normals, normal_buffer, mesh_data.float_list);
    create_index_list(mesh_data.texcoords, texcoord_buffer, mesh_data.float_list);

    state_finish();

    std::set<std::array<uint32_t, 3>, array_compare_less<uint32_t, 3>> triple_set;
    std::set<std::array<uint32_t, 2>, array_compare_less<uint32_t, 2>> double_set;
    state_set("Add vertices to triple list", mesh_data.vertices.size());
    for (const auto &vector : mesh_data.vertices) {
        triple_set.insert(vector);
        state_increase(1);
    }
    state_finish();
    state_set("Add normals to triple list", mesh_data.normals.size());
    for (const auto &vector : mesh_data.normals) {
        triple_set.insert(vector);
        state_increase(1);
    }
    state_finish();
    state_set("Add texture coordinates to double list", mesh_data.texcoords.size());
    for (const auto &vector : mesh_data.texcoords) {
        double_set.insert(vector);
        state_increase(1);
    }
    state_finish();
    state_pause();
    std::vector<std::array<uint32_t, 3>> triple_list;
    std::vector<std::array<uint32_t, 2>> double_list;
    std::copy(triple_set.begin(), triple_set.end(), std::inserter(triple_list, triple_list.end()));
    std::copy(double_set.begin(), double_set.end(), std::inserter(double_list, double_list.end()));
    std::vector<uint32_t> vertex_triple_list, normal_triple_list, texcoord_double_list;
    state_set("Create vertex index list", mesh_data.vertices.size());
    for (const auto &vector : mesh_data.vertices) {
        auto location = std::lower_bound(triple_list.begin(), triple_list.end(), vector, decltype(triple_set)::key_compare());
        assert(location != triple_list.end());
        auto index = std::distance(triple_list.begin(), location);
        vertex_triple_list.push_back(index);
        state_increase(1);
    }
    state_finish();
    state_set("Create normal index list", mesh_data.normals.size());
    for (const auto &vector : mesh_data.normals) {
        auto location = std::lower_bound(triple_list.begin(), triple_list.end(), vector, decltype(triple_set)::key_compare());
        assert(location != triple_list.end());
        auto index = std::distance(triple_list.begin(), location);
        normal_triple_list.push_back(index);
        state_increase(1);
    }
    state_finish();
    state_set("Create texture coordinate index list", mesh_data.texcoords.size());
    for (const auto &vector : mesh_data.texcoords) {
        auto location = std::lower_bound(double_list.begin(), double_list.end(), vector, decltype(double_set)::key_compare());
        auto index = std::distance(double_list.begin(), location);
        texcoord_double_list.push_back(index);
        state_increase(1);
    }
    state_finish();
    state_pause();

    std::vector<float> float_data;
    std::vector<uint32_t> int_data;
    std::vector<std::array<uint32_t, 4>> object_data;

    std::cerr << "Building buffer data..." << std::endl;

    object_data.push_back(std::array<uint32_t, 4>{ 0, 0, 0, 0 });
    uint32_t float_list_index = object_data.size();
    object_data.push_back(std::array<uint32_t, 4>{ 101, static_cast<uint32_t>(mesh_data.float_list.size()), 0, static_cast<uint32_t>(float_data.size()) });
    std::copy(mesh_data.float_list.begin(), mesh_data.float_list.end(), std::inserter(float_data, float_data.end()));
    uint32_t double_list_index = object_data.size();
    object_data.push_back(std::array<uint32_t, 4>{ 102, static_cast<uint32_t>(double_list.size()), static_cast<uint32_t>(int_data.size()), float_list_index });
    for (const auto &vector : double_list) {
        for (const auto &value : vector) {
            int_data.push_back(value);
        }
    }
    uint32_t triple_list_index = object_data.size();
    object_data.push_back(std::array<uint32_t, 4>{ 103, static_cast<uint32_t>(triple_list.size()), static_cast<uint32_t>(int_data.size()), float_list_index });
    for (const auto &vector : triple_list) {
        for (const auto &value : vector) {
            int_data.push_back(value);
        }
    }
    uint32_t vertex_triple_list_index = object_data.size();
    object_data.push_back(std::array<uint32_t, 4>{ 33, static_cast<uint32_t>(vertex_triple_list.size()), static_cast<uint32_t>(int_data.size()), triple_list_index });
    std::copy(vertex_triple_list.begin(), vertex_triple_list.end(), std::inserter(int_data, int_data.end()));
    uint32_t normal_triple_list_index = object_data.size();
    object_data.push_back(std::array<uint32_t, 4>{ 33, static_cast<uint32_t>(normal_triple_list.size()), static_cast<uint32_t>(int_data.size()), triple_list_index });
    std::copy(normal_triple_list.begin(), normal_triple_list.end(), std::inserter(int_data, int_data.end()));
    uint32_t texcoord_double_list_index = object_data.size();
    object_data.push_back(std::array<uint32_t, 4>{ 32, static_cast<uint32_t>(texcoord_double_list.size()), static_cast<uint32_t>(int_data.size()), double_list_index });
    std::copy(texcoord_double_list.begin(), texcoord_double_list.end(), std::inserter(int_data, int_data.end()));

    // TODO: Instead of "3", use flags to specify attributes: position (always included), normal, texcoord, tangent and cotangent
    // TODO: Flags may also specify additional information: textures for the mesh, etc. If present it might modify how int_data and float_data is read.
    uint32_t triangle_data_index = object_data.size();
    object_data.push_back(std::array<uint32_t, 4>{ 30, static_cast<uint32_t>(mesh_data.triangles.size()), static_cast<uint32_t>(int_data.size()), 3 /* number of attributes per triangle vertex */ });
    int_data.push_back(vertex_triple_list_index); // Index for the first attribute (position)
    int_data.push_back(normal_triple_list_index); // Index for the second attribute (normals)
    int_data.push_back(texcoord_double_list_index); // Index for the third attribute
    for (const auto &triangle : mesh_data.triangles) {
        for (const auto &vertex : triangle) {
            for (const auto &attribute : vertex) {
                int_data.push_back(attribute);
            }
        }
    }

    state_set("Generating index nodes");
    std::vector<uint32_t> selection_list(static_cast<std::size_t>(triangle_count.value));
    std::iota(selection_list.begin(), selection_list.end(), 0);
    auto root_node = compile_node(mesh_data, selection_list);

    state_finish();

    state_set("Inserting index nodes into the buffer data", root_node->node_count);
    auto root_index = insert_node(float_data, int_data, object_data, root_node);
    state_finish();
    state_pause();

    std::cerr << "Generating output..." << std::endl;
    std::cerr << "Stats:" << std::endl;
    std::cerr << "  - float_data: " << float_data.size() << " entries" << std::endl;
    std::cerr << "  - int_data: " << int_data.size() << " entries" << std::endl;
    std::cerr << "  - object_data: " << object_data.size() << " entries" << std::endl;
    std::cerr << "  - node_count: " << root_node->node_count << " nodes" << std::endl;
    std::cerr << "  - max_depth: " << root_node->max_depth << " levels" << std::endl;

    BufferSize float_data_size, int_data_size, object_data_size, root_node_index, tree_depth;
    float_data_size.value = float_data.size();
    int_data_size.value = int_data.size();
    object_data_size.value = object_data.size();
    root_node_index.value = root_index;
    tree_depth.value = root_node->max_depth;

    std::cout.write(float_data_size.bytes, sizeof(BufferSize));
    std::cout.write(int_data_size.bytes, sizeof(BufferSize));
    std::cout.write(object_data_size.bytes, sizeof(BufferSize));
    std::cout.write(root_node_index.bytes, sizeof(BufferSize));
    std::cout.write(tree_depth.bytes, sizeof(BufferSize));
    std::cout.write(reinterpret_cast<const char *>(float_data.data()), float_data.size() * sizeof(decltype(float_data)::value_type));
    std::cout.write(reinterpret_cast<const char *>(int_data.data()), int_data.size() * sizeof(decltype(int_data)::value_type));
    std::cout.write(reinterpret_cast<const char *>(object_data.data()), object_data.size() * sizeof(decltype(object_data)::value_type));

    std::cerr << "Done." << std::endl;

    return 0;
}