#include "lightpq.hpp"
#include "grid2d.hpp" // Full definition required here for implementation

namespace planning_module
{
    lightpq::lightpq(grid2d &g)
        : grid(&g),
          U(std::make_unique<std::pair<coordf, uint32_t>[]>(g.size().first * g.size().second))
    {
    }

    void lightpq::swap(uint32_t i, uint32_t j)
    {
        std::swap(U[i], U[j]);
        (*grid)[U[i].second].heap_index = i;
        (*grid)[U[j].second].heap_index = j;
    }

    void lightpq::siftup(uint32_t idx)
    {
        while (idx > 0)
        {
            uint32_t parent = (idx - 1) / 2;
            if (std::tie(U[idx].first) < std::tie(U[parent].first))
            {
                swap(idx, parent);
                idx = parent;
            }
            else
                return;
        }
    }

    void lightpq::siftdown(uint32_t idx)
    {
        uint32_t child = 2 * idx + 1;
        while (child < U_size)
        {
            if (child + 1 < U_size && std::tie(U[child + 1].first) < std::tie(U[child].first))
                child++;
            if (std::tie(U[idx].first) < std::tie(U[child].first))
                return;
            swap(idx, child);
            idx = child;

            child = 2 * idx + 1;
        }
    }

    void lightpq::push(std::pair<coordf, coordi> element)
    {
        uint32_t h_idx = (*grid)[element.second].heap_index;
        uint32_t g_idx = grid->index(element.second);

        if (h_idx == -1)
        {
            U[U_size] = {element.first, g_idx};
            (*grid)[element.second].heap_index = U_size;
            siftup(U_size++);
        }
        else
        {
            U[h_idx] = {element.first, g_idx};
            siftup(h_idx);
            siftdown(h_idx);
        }
    }

    std::pair<coordf, coordi> lightpq::pop()
    {
        if (U_size == 0)
            return {{INFTY, INFTY}, coordi{0, 0}}; // or throw an exception
        auto result = U[0];
        (*grid)[result.second].heap_index = -1;
        
        if (--U_size > 0)
        {
            U[0] = U[U_size];
            (*grid)[U[0].second].heap_index = 0;
            siftdown(0);
        }
        
        return {result.first, grid->coord_from_index(result.second)}; // return the coordinate instead of the index
    }
}
