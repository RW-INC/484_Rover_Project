#ifndef __GRID_2D__
#define __GRID_2D__

#include <memory>
#include <utility>
#include <vector>
#include "planning.hpp"

namespace planning_module
{
    class grid2d
    {
    private:
        std::unique_ptr<datapoint[]> data;
        uint32_t rows;
        uint32_t cols;

    public:
        grid2d(uint32_t r, uint32_t c,
               const float *elevation,
               const float *illumination,
               const bool *is_obstacle);

        template <typename U>
        constexpr inline uint32_t index(const Vec2<U> &c) const
        {
            return static_cast<uint32_t>(c.y) * this->cols + static_cast<uint32_t>(c.x);
        }

        constexpr inline coordi coord_from_index(const uint32_t idx) const
        {
            return {static_cast<int32_t>(idx % this->cols),
                    static_cast<int32_t>(idx / this->cols)};
        }

        template <typename U>
        inline datapoint &operator[](const Vec2<U> &c) { return data[index(c)]; }
        inline datapoint &operator[](const uint32_t idx) { return data[idx]; }

        template <typename U>
        inline const datapoint &operator[](const Vec2<U> &c) const { return data[index(c)]; }
        inline const datapoint &operator[](const uint32_t idx) const { return data[idx]; }

        inline bool is_obstacle(coordi c) const { return data[index(c)].is_obstacle; }

        constexpr std::pair<uint32_t, uint32_t> size() const { return {this->rows, this->cols}; }

        // Reset planner state (g, rhs, heap_index, has_visited) without clobbering
        // elevation/illumination/obstacle data. Useful between replans.
        void reset_search_state();
    };
};

#endif