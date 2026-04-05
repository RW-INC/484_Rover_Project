#include <iostream>
#include <vector>
#include "../Planning/planning.hpp"
#include "../Planning/d_star.hpp"
#include "../Utils/plotter.hpp"

#include <random>

int main()
{
    std::cout << "--- SPARX Windows Test ---" << std::endl;

    // lets create some random grid2d for testing.
    // it seems i need 3 disparate arrays for this, so here we are.
    uint32_t rows = 100, cols = 100;

    // easiest possible testcase. lol
    std::vector<float_t> elevation(rows * cols, 0);

    std::mt19937 rng(117);
    std::uniform_int_distribution<uint32_t> dist(0, 100);
    for (int i = 0; i < rows * cols; ++i)
    {
        elevation[i] = dist(rng);
    }

    std::vector<float_t> illumination(rows * cols, 0);
    std::vector<uint8_t> obstacles(rows * cols, 0);

    // make the grid2d object on the stack
    planning_module::grid2d grid(
        rows,
        cols,
        elevation.data(),
        illumination.data(),
        (bool *)obstacles.data() // yucky.
    );

    planning_module::d_star p(
        rows, cols,
        planning_module::coordi{0, 0},
        planning_module::coordi{99, 99},
        planning_module::coordi{0, 0},
        std::move(grid) // also yucky, but whatever we use unique_ptrs so its fast
    );

    p.compute_shortest_path();
    auto [path, path_length] = p.extract_path();

    printf("Path length: %u\n", path_length);

    for (uint32_t i = 0; i < path_length; i++)
    {
        std::cout << "Step " << i << ": (" << path[i].x << ", " << path[i].y << ")" << std::endl;
    }
    // save the heatmaps
    plotter_module::save_heatmap("g_values.ppm", cols, rows, elevation.data(), path, path_length);

    return 0;
}