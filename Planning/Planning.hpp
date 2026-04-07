#ifndef __PLANNING_MODULE__
#define __PLANNING_MODULE__

#include <vector>
#include <utility>
#include <deque>
#include <map>
#include <queue>
#include <memory>
#include <array>

#include <cmath>
#include <algorithm>
#include <functional>

// units: cm

/**
 * Just a module of functions.
 */
#define INFTY std::numeric_limits<float>::infinity()
namespace planning_module
{
    template <typename T>
    struct Vec2
    {
        T x, y;

        template <typename U>
        constexpr Vec2(const Vec2<U> &other) : x(static_cast<T>(other.x)), y(static_cast<T>(other.y)) {}
        constexpr Vec2(T x = 0, T y = 0) : x(x), y(y) {}

        bool operator<(const Vec2 &other) const { return std::tie(x, y) < std::tie(other.x, other.y); }
        bool operator==(const Vec2 &other) const { return x == other.x && y == other.y; }
        bool operator!=(const Vec2 &other) const { return !(*this == other); }

        // take advantage of promotion rules!
        template <typename U>
        auto operator+(const Vec2<U> &other) -> Vec2<decltype(x + other.x)> { return {x + other.x, y + other.y}; }
        template <typename U>
        auto operator-(const Vec2<U> &other) -> Vec2<decltype(x - other.x)> { return {x - other.x, y - other.y}; }

        template <typename U>
        auto operator*(const Vec2<U> &other) -> Vec2<decltype(x * other.x)> { return {x * other.x, y * other.y}; }
        template <typename U>
        auto operator/(const Vec2<U> &other) -> Vec2<decltype(x / other.x)> { return {x / other.x, y / other.y}; }

        // scalar ops
        template <typename U>
        auto operator*(const U &scalar) -> Vec2<decltype(x * scalar)> { return {x * scalar, y * scalar}; }
        template <typename U>
        auto operator/(const U &scalar) -> Vec2<decltype(x / scalar)> { return {x / scalar, y / scalar}; }

        template <typename U>
        constexpr auto dot(const Vec2<U> &other) -> decltype(x * other.x) { return x * other.x + y * other.y; }
        constexpr auto round() -> Vec2<int32_t> { return Vec2<int32_t>(static_cast<int32_t>(std::round(x)),
                                                                       static_cast<int32_t>(std::round(y))); }
    };

    using coordi = Vec2<int32_t>;
    using coordf = Vec2<float_t>;
    inline bool float_eq(float_t a, float_t b, float_t epsilon = 1e-5) { return  ((!std::isinf(a) && !std::isinf(b)) && std::abs(a - b) < epsilon) || (a == b); }
    
    // 20 bytes :(
    struct datapoint
    {
        float_t elevation;
        float_t illumnation;
        float_t g;
        float_t rhs;
        int32_t heap_index: 30; 
        bool has_visited: 1; 
        bool is_obstacle: 1;    // 0 -> obstacle, 1 -> free space
    };
};

#endif