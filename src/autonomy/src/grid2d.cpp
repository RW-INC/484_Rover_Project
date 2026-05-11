#include "grid2d.hpp"

namespace planning_module
{
    grid2d::grid2d(uint32_t r, uint32_t c,
                   const float *elevation,
                   const float *illumination,
                   const bool *is_obstacle)
        : data(std::make_unique<datapoint[]>(r * c)), rows(r), cols(c)
    {
        for (uint32_t y = 0; y < rows; y++)
        {
            for (uint32_t x = 0; x < cols; x++)
            {
                uint32_t i = y * cols + x;
                data[i].elevation = elevation[i];
                data[i].illumnation = illumination[i];
                data[i].is_obstacle = is_obstacle[i];
                data[i].dist_to_spline = 0.0f;
                data[i].g = INFTY;
                data[i].rhs = INFTY;
                data[i].heap_index = -1;
                data[i].has_visited = false;
            }
        }
    }

    void grid2d::reset_search_state()
    {
        const uint32_t n = rows * cols;
        for (uint32_t i = 0; i < n; i++)
        {
            data[i].g = INFTY;
            data[i].rhs = INFTY;
            data[i].heap_index = -1;
            data[i].has_visited = false;
        }
    }
}