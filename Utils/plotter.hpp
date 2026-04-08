#ifndef __PLOTTER_MODULE__
#define __PLOTTER_MODULE__

#include <fstream>
#include <iostream>
#include <algorithm>
#include "../Planning/Planning.hpp"

/**
 * Just a module of functions.
 */
namespace plotter_module
{
    void save_heatmap(const char* filename, int width, int height, float *grid, const planning_module::coordi *path, int path_length)
    {
        // 1. Create a 1D buffer to hold RGB values for the whole image
        std::vector<uint8_t> img_buffer(width * height * 3);
        auto [min_it, max_it] = std::minmax_element(grid, grid + width * height);
        float min_val = *min_it;
        float max_val = *max_it;
        float range = std::max(max_val - min_val, 1e-6f);

        // 2. Draw the grid (Heatmap background)
        for (int i = 0; i < width * height; ++i)
        {
            float normalized = (grid[i] - min_val) / range;
            uint8_t val = (uint8_t)(255.0f * std::clamp(normalized, 0.0f, 1.0f));
            img_buffer[i * 3 + 0] = val;             // R
            img_buffer[i * 3 + 1] = val;             // G
            img_buffer[i * 3 + 2] = val;             // B
        }

        // 3. Draw the path on top (Overwrite specific pixels)
        for (int i = 0; i < path_length; ++i)
        {
            int x = path[i].x;
            int y = path[i].y;

            // Bounds check just in case
            if (x >= 0 && x < width && y >= 0 && y < height)
            {
                int idx = (y * width + x) * 3;
                img_buffer[idx + 0] = 255; // Red
                img_buffer[idx + 1] = 0;   // Green
                img_buffer[idx + 2] = 0;   // Blue
            }
        }

        // 4. Dump the final buffer to the sequential PPM file
        std::ofstream img(filename);
        img << "P3\n"
            << width << " " << height << "\n255\n"; // Header

        for (size_t i = 0; i < img_buffer.size(); i += 3)
        {
            img << (int)img_buffer[i] << " "
                << (int)img_buffer[i + 1] << " "
                << (int)img_buffer[i + 2] << " ";

            // Optional: newline at the end of each row for readability
            if (((i / 3) + 1) % width == 0)
                img << "\n";
        }
    }
};

#endif
