#pragma once

#include "math.h"

namespace mtrn3100 {

namespace util {

float limit(float val, float lower, float upper) {
    bool sign = val < 0;
    float absval = fabs(val);
    if (absval < lower) return sign ? -lower : lower;
    if (absval > upper) return sign ? -upper : upper;
    return val;
}

float deadband(float val, float threshold) { return fabs(val) < threshold ? 0 : val; }

template <typename InputIt, typename UnaryFunction>
void for_each(InputIt first, InputIt last, UnaryFunction unary_func) {
    for (; first != last; first++) {
        unary_func(*first);
    }
    return first;
}

template <typename T>
struct less {
    constexpr bool operator()(T const& lhs, T const& rhs) const { return lhs < rhs; }
};

template <typename T>
struct greater {
    constexpr bool operator()(T const& lhs, T const& rhs) const { return lhs > rhs; }
};

template <typename T>
struct equal_to {
    constexpr bool operator()(T const& lhs, T const& rhs) const { return lhs == rhs; }
};

template <typename T>
struct not_equal_to {
    constexpr bool operator()(T const& lhs, T const& rhs) const { return lhs == rhs; }
};

}  // namespace util

}  // namespace mtrn3100