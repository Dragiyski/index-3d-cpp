#ifndef COMPILE_MESH_FLOAT_HXX
#define COMPILE_MESH_FLOAT_HXX

#include <cmath>
#include <limits>

template <typename T>
struct float_is_equal {
    bool operator()(T a, T b) const {
        static const constexpr auto epsilon = std::numeric_limits<T>::epsilon();
        static const constexpr auto min_value = std::numeric_limits<T>::min();
        static const constexpr auto max_value = std::numeric_limits<T>::max();
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

#endif /* COMPILE_MESH_FLOAT_HXX */