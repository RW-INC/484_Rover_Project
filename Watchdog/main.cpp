#include <iostream>

#include "../Planning/simulation.hpp"
#include "../libtorch_cpu/libtorch/include/ATen/FuncTorchTLS.h"
int main()
{
    constexpr uint32_t terrain_resolution = 300;
    constexpr double trajectory_dt = 0.01;

    simulation::trajectory traj(
        0.0, 0.0,
        1.0, 5.0,
        1.0,
        20, 50);

    simulation::terrain lunar_surface(terrain_resolution, traj, trajectory_dt);

    lunar_surface.write_to_file("terrain.bin");
    lunar_surface.write_projected_trajectory_to_file("terrain_traj.bin");

    std::cout << "Wrote terrain.bin and terrain_traj.bin." << std::endl;
    std::cout << "Plot with: python Watchdog/visualize.py terrain.bin" << std::endl;
    return 0;
}
