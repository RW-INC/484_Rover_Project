#ifndef __PLANNING_MODULE__
#define __PLANNING_MODULE__

#include <vector>
#include <utility>
#include <deque>
#include <map>
#include <queue>
#include <memory>
#include <array>
#include <limits>

#include <cmath>
#include <algorithm>
#include <functional>

// units: cm

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

        // const-correct so they work on temporaries/const refs
        template <typename U>
        auto operator+(const Vec2<U> &other) const -> Vec2<decltype(x + other.x)> { return {x + other.x, y + other.y}; }
        template <typename U>
        auto operator-(const Vec2<U> &other) const -> Vec2<decltype(x - other.x)> { return {x - other.x, y - other.y}; }
        template <typename U>
        auto operator*(const Vec2<U> &other) const -> Vec2<decltype(x * other.x)> { return {x * other.x, y * other.y}; }
        template <typename U>
        auto operator/(const Vec2<U> &other) const -> Vec2<decltype(x / other.x)> { return {x / other.x, y / other.y}; }

        template <typename U>
        auto operator*(const U &scalar) const -> Vec2<decltype(x * scalar)> { return {x * scalar, y * scalar}; }
        template <typename U>
        auto operator/(const U &scalar) const -> Vec2<decltype(x / scalar)> { return {x / scalar, y / scalar}; }

        template <typename U>
        constexpr auto dot(const Vec2<U> &other) const -> decltype(x * other.x) { return x * other.x + y * other.y; }

        constexpr auto round() const -> Vec2<int32_t>
        {
            return Vec2<int32_t>(static_cast<int32_t>(std::round(x)),
                                 static_cast<int32_t>(std::round(y)));
        }
    };

    using coordi = Vec2<int32_t>;
    using coordf = Vec2<float>;

    inline bool float_eq(float a, float b, float epsilon = 1e-5f)
    {
        if (a == b)
            return true; // handles inf == inf
        return std::abs(a - b) < epsilon;
    }

    // per-cell payload. Removed unused x,y fields.
    struct datapoint
    {
        float dist_to_spline; // spline-cost penalty (see d_star::update_grid)
        float elevation;
        float illumnation;
        float g;
        float rhs;
        int32_t heap_index : 30; // -1 = not in heap
        bool has_visited : 1;
        bool is_obstacle : 1;
    };
};

#endif