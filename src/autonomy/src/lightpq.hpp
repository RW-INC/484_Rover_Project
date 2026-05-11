#ifndef __LIGHTPQ__
#define __LIGHTPQ__

#include <memory>
#include <utility>
#include "planning.hpp"

namespace planning_module
{
    class grid2d;

    class lightpq
    {
    private:
        std::unique_ptr<std::pair<coordf, uint32_t>[]> U;
        std::shared_ptr<grid2d> grid;
        uint32_t U_size = 0;

        void swap(uint32_t i, uint32_t j);
        void siftup(uint32_t idx);
        void siftdown(uint32_t idx);

    public:
        explicit lightpq(std::shared_ptr<grid2d> g);

        const std::pair<coordf, uint32_t> &operator[](uint32_t idx) const { return U[idx]; }
        uint32_t size() const { return U_size; }

        void push(std::pair<coordf, coordi> element);
        std::pair<coordf, coordi> pop();
    };
}

#endif