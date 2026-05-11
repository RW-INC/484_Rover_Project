#include "lightpq.hpp"
#include "grid2d.hpp"

namespace planning_module
{
    lightpq::lightpq(std::shared_ptr<grid2d> g)
        : U(std::make_unique<std::pair<coordf, uint32_t>[]>(g->size().first * g->size().second)),
          grid(std::move(g))
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
            if (U[idx].first < U[parent].first)
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
            // pick smaller of the two children
            if (child + 1 < U_size && U[child + 1].first < U[child].first)
                child++;
            // if parent already <= smaller child, we're done
            if (!(U[child].first < U[idx].first))
                return;
            swap(idx, child);
            idx = child;
            child = 2 * idx + 1;
        }
    }

    void lightpq::push(std::pair<coordf, coordi> element)
    {
        int32_t h_idx = (*grid)[element.second].heap_index;
        uint32_t g_idx = grid->index(element.second);

        if (h_idx == -1)
        {
            U[U_size] = {element.first, g_idx};
            (*grid)[element.second].heap_index = static_cast<int32_t>(U_size);
            siftup(U_size++);
        }
        else
        {
            // already in heap: update key, then re-heapify both ways
            U[h_idx] = {element.first, g_idx};
            siftup(h_idx);
            siftdown((*grid)[element.second].heap_index); // index may have changed after siftup
        }
    }

    std::pair<coordf, coordi> lightpq::pop()
    {
        if (U_size == 0)
            return {{INFTY, INFTY}, coordi{0, 0}};

        auto result = U[0];
        (*grid)[result.second].heap_index = -1;

        if (--U_size > 0)
        {
            U[0] = U[U_size];
            (*grid)[U[0].second].heap_index = 0;
            siftdown(0);
        }

        return {result.first, grid->coord_from_index(result.second)};
    }
}