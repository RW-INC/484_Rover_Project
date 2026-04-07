#ifndef __GRID_2D__
#define __GRID_2D__

#include <memory>
#include <utility>
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
        /**
         * Modern c++ constructor holy efficient.
         */
        grid2d(uint32_t r, uint32_t c, float elevation[], float illumination[], bool is_obstacle[]);

        /**
         * Get the index in the 1D data array corresponding to a given coordinate.
         * @param c The coordinate to convert.
         * @return The index in the data array corresponding to the coordinate.
         * */
        template <typename U>
        constexpr inline uint32_t index(const Vec2<U> &c) const
        {
            return static_cast<uint32_t>(c.y) * this->cols + static_cast<uint32_t>(c.x);
        }

        /**
         * Get the coordinate corresponding to a given index in the 1D data array.
         * @param idx The index to convert.
         * @return The coordinate corresponding to the index in the data array.
         */
        constexpr inline coordi coord_from_index(const uint32_t idx) const
        {
            return {static_cast<int32_t>(idx % this->cols), static_cast<int32_t>(idx / this->cols)};
        }

        /**
         * Make my life easier operator.
         */
        template <typename U>
        inline datapoint &operator[](const Vec2<U> &c) { return data[index(c)]; }
        inline datapoint &operator[](const uint32_t idx) { return data[idx]; }

        /**
         * Determines whether a given coordinate is an obstacle on the grid.
         * @param c The coordinate to check.
         * @return Whether the coordinate is an obstacle on the grid.
         */
        inline bool is_obstacle(coordi c) { return data[index(c)].is_obstacle; }

        /**
         * Gets the size of the grid as a pair of (rows, cols).
         * @return A pair of uint32_t representing the number of rows and columns in the grid.
         */
        constexpr std::pair<uint32_t, uint32_t> size() const { return {this->rows, this->cols}; }
    };
};

#endif