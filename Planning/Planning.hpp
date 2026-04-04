

#ifndef __PLANNING_MODULE__
#define __PLANNING_MODULE__

#include <vector>
#include <utility>
#include <deque>
#include <map>
#include <queue>
#include <memory>
#include <array>

#include <cmath>
#include <algorithm>
#include <functional>

// units: cm

/**
 * Just a module of functions.
 */
#define INTEGER_MAX std::numeric_limits<float>::infinity()
namespace planning_module
{
    template <typename T>
    struct Vec2
    {
        T x, y;

        template <typename U>
        constexpr Vec2(const Vec2<U> &other) : x(static_cast<T>(other.x)), y(static_cast<T>(other.y)) {}
        constexpr Vec2(T x = 0, T y = 0) : x(x), y(y) {}

        bool operator<(const Vec2 &other) const { return std::tie(x, y) < std::tie(other.x, other.y); }
        bool operator==(const Vec2 &other) const { return x == other.x && y == other.y; }
        bool operator!=(const Vec2 &other) const { return !(*this == other); }

        // take advantage of promotion rules!
        template <typename U>
        auto operator+(const Vec2<U> &other) -> Vec2<decltype(x + other.x)> { return {x + other.x, y + other.y}; }
        template <typename U>
        auto operator-(const Vec2<U> &other) -> Vec2<decltype(x - other.x)> { return {x - other.x, y - other.y}; }

        template <typename U>
        auto operator*(const Vec2<U> &other) -> Vec2<decltype(x * other.x)> { return {x * other.x, y * other.y}; }
        template <typename U>
        auto operator/(const Vec2<U> &other) -> Vec2<decltype(x / other.x)> { return {x / other.x, y / other.y}; }

        // scalar ops
        template <typename U>
        auto operator*(const U &scalar) -> Vec2<decltype(x * scalar)> { return {x * scalar, y * scalar}; }
        template <typename U>
        auto operator/(const U &scalar) -> Vec2<decltype(x / scalar)> { return {x / scalar, y / scalar}; }

        template <typename U>
        constexpr auto dot(const Vec2<U> &other) -> decltype(x * other.x) { return x * other.x + y * other.y; }
        constexpr auto round() -> Vec2<int32_t> { return Vec2<int32_t>(static_cast<int32_t>(std::round(x)),
                                                             static_cast<int32_t>(std::round(y))); }
    };

    using coordi = Vec2<int32_t>;
    using coordf = Vec2<float_t>;

    namespace d_star
    {
        struct datapoint
        {
            float_t elevation;
            float_t illumnation;
            float_t g;
            float_t rhs;
            bool has_visited = false;
            bool is_obstacle; // 0 -> obstacle, 1 -> free space
        };

        class grid2d
        {
        private:
            std::unique_ptr<datapoint[]> data;
            uint32_t rows;
            uint32_t cols;

        public:
            /**
             * Modern c++ constructor holy efficient.
             * @param r The number of rows in the grid.
             * @param c The number of columns in the grid.
             * @param elevation An array of elevations for each cell in the grid.
             * @param illumnation An array of illuminations for each cell in the grid.
             * @param is_obstacle An array indicating whether each cell in the grid is an obstacle (0) or free space (1).
             * Initializes a 2D grid of datapoints with the given number of rows and columns, using a unique pointer for memory management.
             */
            grid2d(uint32_t r, uint32_t c, uint32_t elevation[], uint32_t illumnation[], bool is_obstacle[])
                : data(std::make_unique<datapoint[]>(r * c)), rows(r), cols(c)
            {
                for (uint32_t i = 0; i < this->rows * this->cols; i++)
                {
                    data[i].elevation = elevation[i];
                    data[i].illumnation = illumnation[i];
                    data[i].is_obstacle = is_obstacle[i];

                    data[i].g = INTEGER_MAX;
                    data[i].rhs = INTEGER_MAX;
                }
            }

            /**
             * Make my life easier operator.
             * @param c The coordinate to index into the grid.
             * @return A pointer to the datapoint at the given coordinate.
             */

            datapoint &operator[](coordi c) { 
                return this->data[(uint32_t)(c.dot(Vec2<uint32_t>(this->cols, 1)))]; 
            }


            /**
             * Determines whether a given coordinate is an obstacle on the grid.
             * @param grid The grid to check on.
             * @param c The coordinate to check.
             * @return Whether the coordinate is an obstacle.
             */
            bool is_obstacle(coordi c) { return (*this)[c].is_obstacle == 0; }

            /**
             * Gets the size of the grid as a pair of (rows, cols).
             * @return A pair of (rows, cols) representing the size of the grid.
             */
            std::pair<uint32_t, uint32_t> size() { return {this->rows, this->cols}; }
        };

        class d_star
        {
        private:
            uint32_t rows;
            uint32_t cols;
            coordi start;
            coordi goal;
            coordi lander_loc;
            grid2d grid;

            constexpr static std::array<coordi, 8> directions{
                coordi{1, 0}, coordi{0, 1}, coordi{-1, 0}, coordi{0, -1},
                coordi{1, 1}, coordi{1, -1}, coordi{-1, 1}, coordi{-1, -1}};

            float km = 0;                                   // travel dist
            std::unique_ptr<std::pair<coordf, coordi>[]> U; // we require pq type pop and push behavior, yet we want to delete
                                                            // arbitrary elements. So we need to use a red-black tree.
            uint32_t U_size = 0;                            // track the size of the queue since we are using a heap on an array
        public:
            /**
             * Rule of zero. Will steal grid2d. don't expect it back.
             */
            d_star(uint32_t rows, uint32_t cols,
                   coordi start, coordi goal, coordi lander_loc, grid2d grid)
                : rows(rows), cols(cols), start(start), goal(goal), lander_loc(lander_loc), grid(std::move(grid)),
                  U(std::make_unique<std::pair<coordf, coordi>[]>((uint32_t)(2ULL * rows * cols))) // allocate more than we need to avoid overflow, since we are being lazy and not deleting elements from the queue
            {
            }

            /**
             * Computes the shortest path.
             */
            void compute_shortest_path()
            {
                auto start_key = this->calculate_key(this->start);

                while (this->U_size > 0 && (this->U[0].first < start_key || this->grid[this->start].rhs != this->grid[this->start].g))
                {
                    // move U[0] to the end (logn time, we're fine for efficiency)
                    std::pop_heap(this->U.get(), this->U.get() + this->U_size, std::greater<>{});
                    auto [k_old, u] = this->U[--this->U_size]; //"pop" the element
                    auto k_new = this->calculate_key(u);

                    // push to the queue, continue.
                    if (k_old < k_new)
                    {
                        this->U[U_size++] = {k_new, u};
                        std::push_heap(this->U.get(), this->U.get() + this->U_size, std::greater<>{});
                        continue;
                    }

                    // otherwise, we gotta compute some shit
                    bool inconsistent = (this->grid[u].g > this->grid[u].rhs);
                    this->grid[u].g = (inconsistent) ? this->grid[u].rhs : INTEGER_MAX;
                    auto [ns, count] = this->neighbors(u, !inconsistent);

                    for (uint32_t i = 0; i < count; i++)
                        this->update_vertex(ns[i]);
                }
            }

            /**
             * Updates the vertex u by recalculating its rhs value based on the costs of its neighbors and adding it back into the priority queue U if its g value is not equal to its rhs value. If u is the goal, its rhs value is not updated.
             * @param u The coordinate of the vertex to update.
             */
            void update_vertex(coordi u)
            {
                if (u != this->goal)
                {
                    auto [ns, count] = this->neighbors(u);

                    float_t rhs = (float_t)INTEGER_MAX;
                    for (uint32_t i = 0; i < count; i++)
                        rhs = std::min(rhs, this->cost(u, ns[i]) + this->grid[ns[i]].g);

                    this->grid[u].rhs = rhs;
                }

                if (this->grid[u].g != this->grid[u].rhs)
                {
                    // instead of deleting the element, be lazy and add it back into the queue with a new key.
                    auto key = this->calculate_key(u);
                    U[U_size++] = {key, u};

                    // add (key, u) to the queue with push_heap
                    std::push_heap(this->U.get(), this->U.get() + this->U_size, std::greater<>{});
                }
            }

            /**
             * Calculates the key for a given coordinate based on its g and rhs values, the heuristic distance to the start, and the current km value.
             * @param s The coordinate to calculate the key for.
             * @return A pair of floats representing the key for the given coordinate, where the first element is the sum of the minimum of g and rhs, the heuristic distance to the start, and km, and the second element is the minimum of g and rhs.
             */
            coordf calculate_key(coordi s)
            {
                auto k1 = std::min(this->grid[s].g, this->grid[s].rhs) + heuristic(s, this->start) + this->km; // add 1 to break ties in favor of larger g values
                auto k2 = std::min(this->grid[s].g, this->grid[s].rhs);
                return {k1, k2};
            }

            /**
             * Gets the neighboring coordinates of a given coordinate on the grid, considering only the four cardinal directions (up, down, left, right) and ensuring that the neighbors are within the bounds of the grid.
             * @param s The coordinate to get the neighbors of.
             * @param include_s Whether to include the original coordinate s in the list of neighbors.
             * @return A vector of neighboring coordinates that are within the bounds of the grid.
             */
            std::pair<std::array<coordi, 9>, size_t> neighbors(coordi s, bool include_s = false)
            {
                std::array<coordi, 9> result;
                size_t count = 0;

                for (coordi d : directions)
                {
                    auto next = s + d;
                    if (next.x >= 0 && next.x < this->rows && next.y >= 0 && next.y < this->cols)
                    {
                        result[count++] = next;
                    }
                }

                if (include_s)
                    result[count++] = s;

                return {result, count};
            }

            /**
             * Calculates the cost of moving from one coordinate to another on the grid, taking into account factors such as distance, line of sight, and solar illumination. If either coordinate is an obstacle, the cost is set to a maximum integer value.
             * @param a The starting coordinate.
             * @param b The ending coordinate.
             * @return The calculated cost of moving from coordinate a to coordinate b, which is the sum of the distance cost, line of sight penalty, and solar penalty, or a maximum integer value if either coordinate is an obstacle.
             */
            float_t cost(coordi a, coordi b)
            {
                if (this->grid.is_obstacle(a) || this->grid.is_obstacle(b))
                    return INTEGER_MAX;

                auto diff = b - a;

                // i hate branches.
                // first off, for those who say ternary is better, probably only on x86/x64 architectures
                // with 'cmov', or 'csel' or whatever the hell on non-intel, which STILL isn't guaranteed by the standard.
                // second, this is a trade-off. while strict math is not faster than a register move,
                // microcontroller architectures DON'T HAVE THAT OPERATION!!! GET OFF MY ASS!
                float_t dist_cost = ((abs(diff.x) == 1) & (abs(diff.y) == 1)) * 0.4f + 1.0f;
                float_t los_penalty = (!has_line_of_sight(a, b)) * 0.2f;
                float_t solar_penalty = (this->grid[b].illumnation < 0.5) * 0.5f;

                return dist_cost + los_penalty + solar_penalty;
            }

            /**
             * Calculates the heuristic distance between two coordinates using Manhattan distance.
             * @param a The first coordinate.
             * @param b The second coordinate.
             * @return The Manhattan distance between the two coordinates.
             */
            float_t heuristic(coordi a, coordi b) { return std::abs(a.x - b.x) + std::abs(a.y - b.y); }

            /**
             * Determines whether there is a line of sight between two coordinates on the grid, taking into account elevation changes and obstacles.
             * @param grid The grid to check on.
             * @param a The starting coordinate.
             * @param b The ending coordinate.
             * @return Whether there is a line of sight between the two coordinates.
             */
            bool has_line_of_sight(coordi a, coordi b)
            {
                auto diff = b - a;

                auto z0 = this->grid[a].elevation;
                auto z1 = this->grid[b].elevation;

                auto step_size = std::max(std::abs(diff.x), std::abs(diff.y));

                for (uint32_t i = 0; i <= step_size; i++)
                {
                    auto t = ((float)i) / step_size;

                    auto test_point = (a + diff * t).round();
                    auto z_expected = z0 + t * (z1 - z0);
                    auto z_actual = this->grid[test_point].elevation;

                    if (z_actual > z_expected)
                        return false;
                }
                return true;
            }

            /**
             * Determines whether, given a grid and start postions, a path exists to the goal.
             * @param grid The grid to search on.
             * @param start The starting position.
             * @param goal The goal position.
             * @return Whether a path exists.
             */
            bool path_exists(coordi start, coordi goal)
            {
                auto [rows, cols] = this->grid.size();
                auto rc_pair = std::make_pair(rows, cols);
                auto start_pair = std::make_pair(0, 0);

                auto q = std::deque<coordi>();

                q.push_back(start);

                while (!q.empty())
                {
                    coordi curr = q.front();
                    if (curr == goal)
                        return true;

                    // thank god for c++17 structured bindings
                    for (coordi d : directions)
                    {
                        auto next_pair = curr + d;

                        if (next_pair.x < 0 || next_pair.x >= rc_pair.first ||
                            next_pair.y < 0 || next_pair.y >= rc_pair.second ||
                            this->grid[next_pair].has_visited || this->grid.is_obstacle(next_pair))
                            continue;

                        this->grid[next_pair].has_visited = true;
                        q.push_back(next_pair);
                    }

                    q.pop_front();
                }
                return false;
            }
        };
    };
};

#endif