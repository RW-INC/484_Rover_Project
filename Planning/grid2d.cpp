#include "grid2d.hpp"

namespace planning_module
{
    grid2d::grid2d(uint32_t r, uint32_t c, float elevation[], float illumination[], bool is_obstacle[])
        : data(std::make_unique<datapoint[]>(r * c)), rows(r), cols(c)
    {
        for (uint32_t i = 0; i < this->rows * this->cols; i++)
        {
            data[i].elevation = elevation[i];
            data[i].illumnation = illumination[i];
            data[i].is_obstacle = is_obstacle[i];

            // Initialize D* Lite defaults
            data[i].g = INFTY;
            data[i].rhs = INFTY;
            data[i].heap_index = -1; // Default to "not in heap"
            data[i].has_visited = false;
        }
    }
}