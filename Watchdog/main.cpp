#include <iostream>
#include <vector>
#include "../Planning/planning.hpp"
#include "../Planning/d_star.hpp"
#include "../Utils/plotter.hpp"

#define HAVE_SSTREAM
#include "../spline/src/spline.h"

#include "../Nav/testcase.hpp"

int main()
{
    // make a testcase 
    SimConfig cfg;
    
    for(uint32_t i = 0; i < 5; i++)
        cfg.t_vec.push_back((float_t)i*10);

    auto s = generate_testcases(cfg);

    // push the spline points to the csv
    std::ofstream file("spline_points.csv");

    for (float_t x = s->X_SPLINE_POINTS.front(); x <= s->X_SPLINE_POINTS.back(); x += 0.01f)
    {
        auto y = s->SPLINE_INTERPOLATOR(x);
        file << x << "," << y << std::endl;
    }

    file.close();
    return 0;
}



// int main()
// {
//     std::cout << "--- SPARX Windows Test ---" << std::endl;

//     // lets create some random grid2d for testing.
//     // it seems i need 3 disparate arrays for this, so here we are.
//     uint32_t rows = 500, cols = 500;

//     // easiest possible testcase. lol
//     std::vector<float_t> elevation(rows * cols, 0);
//     std::vector<float_t> illumination(rows * cols, 0);
//     std::vector<uint8_t> obstacles(rows * cols, 0);

//     printf("Initializing grid and planner...\n");
//     // make the grid2d object on the stack
//     planning_module::grid2d grid(
//         rows,
//         cols,
//         elevation.data(),
//         illumination.data(),
//         (bool *)obstacles.data() // yucky.
//     );

//     printf("Grid initialized.\n");
//     planning_module::d_star p(
//         rows, cols,
//         planning_module::coordi{0, 0},
//         planning_module::coordi{200, 499},
//         planning_module::coordi{0, 0},
//         std::move(grid) // also yucky, but whatever we use unique_ptrs so its fast
//     );

//     printf("Computing shortest path...\n");
//     p.compute_shortest_path();
//     auto [path, path_length] = p.extract_path();

//     printf("Path length: %u\n", path_length);

//     for (uint32_t i = 0; i < path_length; i++)
//     {
//         std::cout << "Step " << i << ": (" << path[i].x << ", " << path[i].y << ")" << std::endl;
//     }
//     // save the heatmaps
//     plotter_module::save_heatmap("g_values.ppm", cols, rows, elevation.data(), path, path_length);

//     return 0;
// }
