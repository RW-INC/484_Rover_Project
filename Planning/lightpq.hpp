#ifndef __LIGHTPQ__
#define __LIGHTPQ__

#include <memory>
#include <utility>
#include "planning.hpp"

namespace planning_module {
    // Forward declaration
    class grid2d;

    class lightpq {
    private:
        std::unique_ptr<std::pair<coordf, uint32_t>[]> U;
        grid2d *grid; 
        uint32_t U_size = 0;

        void swap(uint32_t i, uint32_t j);
        void siftup(uint32_t idx);
        void siftdown(uint32_t idx);

    public:
        lightpq(grid2d &grid);
        
        const std::pair<coordf, uint32_t> &operator[](uint32_t idx) const { return U[idx]; }
        uint32_t size() const { return U_size; }

        void push(std::pair<coordf, coordi> element);
        std::pair<coordf, coordi> pop();
    };
}

#endif
