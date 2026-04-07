#ifndef __D_STAR__
#define __D_STAR__

#include "planning.hpp"
#include "grid2d.hpp"
#include "lightpq.hpp"

namespace planning_module
{
    class d_star
    {
    private:
        uint32_t rows;
        uint32_t cols;
        coordi start;
        coordi goal;
        coordi lander_loc;

        grid2d grid; 
        lightpq U;   

        std::unique_ptr<coordi[]> path;
        uint32_t path_length = 0;

        constexpr const static std::array<coordi, 8> directions{
            coordi{1, 0}, coordi{0, 1}, coordi{-1, 0}, coordi{0, -1},
            coordi{1, 1}, coordi{1, -1}, coordi{-1, 1}, coordi{-1, -1}};

        float km = 0; // travel dist

    public:
        /**
         * Rule of zero. Will steal grid2d. don't expect it back.
         */
        d_star(uint32_t rows, uint32_t cols, coordi start, coordi goal, coordi lander_loc, grid2d grid_in)
            : rows(rows), cols(cols), start(start), goal(goal), lander_loc(lander_loc),
              grid(std::move(grid_in)), //  Grid is moved in and fully constructed
              U(this->grid),            //  U safely binds ref
              path(std::make_unique<coordi[]>(rows * cols))
        {
            this->grid[this->goal].rhs = 0;
            auto goal_key = this->calculate_key(this->goal);

            U.push({goal_key, this->goal}); // Look ma, no arrows!
        }

        std::pair<const coordi *, uint32_t> extract_path()
        {
            auto current = this->start;
            auto last = this->start;

            this->path[this->path_length++] = current;

            printf("Extracting path from (%d, %d) to (%d, %d)\n", this->start.x, this->start.y, this->goal.x, this->goal.y);
            
            while (current != this->goal)
            {
                coordi best_s{-1, -1};
                float_t min_val = INFTY;

                std::array<coordi, 9> ns;
                uint32_t count = 0;
                this->neighbors(current, ns, count);

                printf("# of neighbors: %d\n", count);
                for (uint32_t i = 0; i < count; i++)
                {
                    coordi s_next = ns[i];
                    float_t val = std::min(min_val, this->cost(current, s_next) + this->grid[s_next].g);
                    printf("Evaluating neighbor (%d, %d): Cost: %f, g: %f, Total: %f\n", s_next.x, s_next.y, this->cost(current, s_next), this->grid[s_next].g, val);
                    best_s = (val < min_val) ? s_next : best_s;
                    min_val = std::min(val, min_val);
                }

                printf("Current: (%d, %d), Best Next: (%d, %d), Cost: %f\n", current.x, current.y, best_s.x, best_s.y, min_val);
                if (best_s == coordi{-1, -1})
                    return {nullptr, 0}; // no path exists

                current = best_s;
                path[path_length++] = current;

                // update pos in planner
                this->km += this->heuristic(last, current);
                last = current;
                start = current;
            }

            return {path.get(), path_length};
        }

        void compute_shortest_path()
        {
            while (this->U.size() > 0 && (this->U[0].first < this->calculate_key(this->start) || !float_eq(this->grid[this->start].rhs, this->grid[this->start].g)))
            {
                auto [k_old, u] = this->U.pop();
                auto k_new = this->calculate_key(u);

                printf("Processing vertex (%d, %d) with old key: (%f, %f) and new key: (%f, %f)\n", u.x, u.y, k_old.x, k_old.y, k_new.x, k_new.y);

                if (k_old < k_new)
                {
                    this->U.push({k_new, u});
                    continue;
                }

                printf("Grid g and rhs before update: g: %f, rhs: %f\n", this->grid[u].g, this->grid[u].rhs);
                
                bool inconsistent = (this->grid[u].g > this->grid[u].rhs);
                this->grid[u].g = (inconsistent) ? this->grid[u].rhs : INFTY;

                std::array<coordi, 9> ns;
                uint32_t count = 0;
                this->neighbors(u, ns, count, !inconsistent);

                for (uint32_t i = 0; i < count; i++)
                    this->update_vertex(ns[i]);

                printf("Done. U size: %u\n", this->U.size());
            }
        }

        void update_vertex(coordi u)
        {
            if (u != this->goal)
            {
                std::array<coordi, 9> ns;
                uint32_t count = 0;
                this->neighbors(u, ns, count);

                float_t rhs = (float_t)INFTY;
                for (uint32_t i = 0; i < count; i++)
                    rhs = std::min(rhs, this->cost(u, ns[i]) + this->grid[ns[i]].g);

                this->grid[u].rhs = rhs;
            }

            if (!float_eq(this->grid[u].g, this->grid[u].rhs))
            {
                auto key = this->calculate_key(u);
                U.push({key, u});
            }
        }

        coordf calculate_key(coordi s)
        {
            auto k1 = std::min(this->grid[s].g, this->grid[s].rhs) + heuristic(s, this->start) + this->km; 
            auto k2 = std::min(this->grid[s].g, this->grid[s].rhs);
            return {k1, k2};
        }

        void neighbors(coordi s, std::array<coordi, 9> &out, uint32_t &count, bool include_s = false)
        {
            count = 0;
            for (coordi d : directions)
            {
                auto next = s + d;
                if (next.x >= 0 && next.x < this->cols && next.y >= 0 && next.y < this->rows)
                {
                    out[count++] = next;
                }
            }

            if (include_s)
                out[count++] = s;
        }

        float_t cost(coordi a, coordi b)
        {
            if (this->grid.is_obstacle(a) || this->grid.is_obstacle(b))
                return INFTY;

            auto diff = b - a;

            uint32_t sum_dist = (abs(diff.x) == 1) + (abs(diff.y) == 1);
            float_t dist_cost = (sum_dist >> 1) * 0.414f + 1.0f;

            float_t los_penalty = (!has_line_of_sight(a, b)) * 0.2f;
            float_t solar_penalty = (this->grid[b].illumnation < 0.5) * 0.5f;
            float_t elevation_penalty = (this->grid[b].elevation - this->grid[a].elevation) * 1.0f;

            return dist_cost + los_penalty + solar_penalty + elevation_penalty;
        }

        float_t heuristic(coordi a, coordi b) { return std::abs(a.x - b.x) + std::abs(a.y - b.y); }

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

        bool path_exists(coordi start, coordi goal)
        {
            auto [rows_sz, cols_sz] = this->grid.size();
            auto rc_pair = std::make_pair(rows_sz, cols_sz);

            auto q = std::deque<coordi>();
            q.push_back(start);

            while (!q.empty())
            {
                coordi curr = q.front();
                if (curr == goal)
                    return true;

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
#endif
