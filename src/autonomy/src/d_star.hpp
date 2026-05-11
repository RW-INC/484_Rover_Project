#ifndef __D_STAR__
#define __D_STAR__

#include "planning.hpp"
#include "grid2d.hpp"
#include "lightpq.hpp"
#include "Eigen/Dense"

#include <fstream>
#include <string>
#include <cstdio>
#include <cstdint>

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

        std::shared_ptr<grid2d> grid;
        lightpq U;

        std::unique_ptr<coordi[]> path;
        uint32_t path_length = 0;

        // 8-connected neighborhood
        constexpr const static std::array<coordi, 8> directions{
            coordi{1, 0}, coordi{0, 1}, coordi{-1, 0}, coordi{0, -1},
            coordi{1, 1}, coordi{1, -1}, coordi{-1, 1}, coordi{-1, -1}};

        float km = 0.0f; // D* Lite key modifier (accumulates as the robot moves)

    public:
        /**
         * Takes shared ownership of grid; the ROS node and the planner share it.
         */
        d_star(uint32_t rows, uint32_t cols,
               coordi start, coordi goal, coordi lander_loc,
               std::shared_ptr<grid2d> grid_in)
            : rows(rows), cols(cols), start(start), goal(goal), lander_loc(lander_loc),
              grid(std::move(grid_in)),
              U(grid),
              path(std::make_unique<coordi[]>(rows * cols))
        {
            (*this->grid)[this->goal].rhs = 0.0f;
            auto goal_key = this->calculate_key(this->goal);
            U.push({goal_key, this->goal});
        }

        /**
         * Call this AFTER the robot has actually moved one step along the path,
         * to bump km and re-anchor the heuristic. Do NOT do this inside extract_path.
         */
        void on_robot_moved(coordi new_pos)
        {
            this->km += this->heuristic(this->start, new_pos);
            this->start = new_pos;
        }

        /**
         * Greedy descent from start to goal over (cost(s,s') + g(s')).
         * Pure read on planner state — no mutation of km/start/last.
         */
        std::pair<const coordi *, uint32_t> extract_path()
        {
            this->path_length = 0;
            coordi current = this->start;
            this->path[this->path_length++] = current;

            printf("Extracting path from (%d, %d) to (%d, %d)\n",
                   this->start.x, this->start.y, this->goal.x, this->goal.y);

            // safety cap so we never spin forever if the grid is in a weird state
            const uint32_t max_steps = this->rows * this->cols;

            while (current != this->goal && this->path_length < max_steps)
            {
                coordi best_s{-1, -1};
                float min_val = INFTY;

                std::array<coordi, 9> ns;
                uint32_t count = 0;
                this->neighbors(current, ns, count);

                for (uint32_t i = 0; i < count; i++)
                {
                    coordi s_next = ns[i];
                    float candidate = this->cost(current, s_next) + (*this->grid)[s_next].g;
                    if (candidate < min_val)
                    {
                        min_val = candidate;
                        best_s = s_next;
                    }
                }

                if (best_s == coordi{-1, -1} || min_val == INFTY)
                {
                    printf("extract_path: dead end at (%d, %d)\n", current.x, current.y);
                    return {nullptr, 0};
                }

                current = best_s;
                path[path_length++] = current;
            }

            return {path.get(), path_length};
        }

        void compute_shortest_path()
        {
            while (this->U.size() > 0 &&
                   (this->U[0].first < this->calculate_key(this->start) ||
                    !float_eq((*this->grid)[this->start].rhs, (*this->grid)[this->start].g)))
            {
                auto top = this->U.pop();
                coordf k_old = top.first;
                coordi u = top.second;

                coordf k_new = this->calculate_key(u);

                if (k_old < k_new)
                {
                    // key was stale; reinsert with updated key
                    this->U.push({k_new, u});
                    continue;
                }

                bool overconsistent = ((*this->grid)[u].g > (*this->grid)[u].rhs);

                if (overconsistent)
                {
                    // g := rhs, then update predecessors only
                    (*this->grid)[u].g = (*this->grid)[u].rhs;

                    std::array<coordi, 9> ns;
                    uint32_t count = 0;
                    this->neighbors(u, ns, count, /*include_s=*/false);
                    for (uint32_t i = 0; i < count; i++)
                        this->update_vertex(ns[i]);
                }
                else
                {
                    // underconsistent: g := infty, update predecessors AND u itself
                    (*this->grid)[u].g = INFTY;

                    std::array<coordi, 9> ns;
                    uint32_t count = 0;
                    this->neighbors(u, ns, count, /*include_s=*/true);
                    for (uint32_t i = 0; i < count; i++)
                        this->update_vertex(ns[i]);
                }
            }
        }

        void update_vertex(coordi u)
        {
            if (u != this->goal)
            {
                std::array<coordi, 9> ns;
                uint32_t count = 0;
                this->neighbors(u, ns, count);

                float rhs = INFTY;
                for (uint32_t i = 0; i < count; i++)
                {
                    float c = this->cost(u, ns[i]) + (*this->grid)[ns[i]].g;
                    if (c < rhs) rhs = c;
                }
                (*this->grid)[u].rhs = rhs;
            }

            if (!float_eq((*this->grid)[u].g, (*this->grid)[u].rhs))
            {
                auto key = this->calculate_key(u);
                U.push({key, u}); // push handles "already in heap" via heap_index
            }
        }

        coordf calculate_key(coordi s)
        {
            float min_g_rhs = std::min((*this->grid)[s].g, (*this->grid)[s].rhs);
            float k1 = min_g_rhs + heuristic(s, this->start) + this->km;
            float k2 = min_g_rhs;
            return {k1, k2};
        }

        void neighbors(coordi s, std::array<coordi, 9> &out, uint32_t &count, bool include_s = false)
        {
            count = 0;
            for (coordi d : directions)
            {
                coordi next = s + d;
                if (next.x >= 0 && next.x < (int32_t)this->cols &&
                    next.y >= 0 && next.y < (int32_t)this->rows)
                {
                    out[count++] = next;
                }
            }
            if (include_s)
                out[count++] = s;
        }

        float cost(coordi a, coordi b)
        {
            if (this->grid->is_obstacle(a) || this->grid->is_obstacle(b))
                return INFTY;

            // Octile step distance: 1 for cardinal, sqrt(2) for diagonal
            auto diff = b - a;
            bool diagonal = (std::abs(diff.x) == 1) && (std::abs(diff.y) == 1);
            float dist_cost = diagonal ? 1.41421356f : 1.0f;

            // Spline-following bias: cells far from the spline cost more
            float spline_penalty = (*this->grid)[b].dist_to_spline;

            // Solar / illumination penalty
            float solar_penalty = ((*this->grid)[b].illumnation < 0.5f) ? 0.5f : 0.0f;

            // Elevation: only penalize uphill (downhill is free; no negative weights)
            float dz = (*this->grid)[b].elevation - (*this->grid)[a].elevation;
            float elevation_penalty = (dz > 0.0f) ? dz : 0.0f;

            return dist_cost + spline_penalty + solar_penalty + elevation_penalty;
        }

        float heuristic(coordi a, coordi b)
        {
            // Octile heuristic — admissible for 8-connected uniform cost.
            // Manhattan would over-estimate diagonal moves and break consistency.
            float dx = static_cast<float>(std::abs(a.x - b.x));
            float dy = static_cast<float>(std::abs(a.y - b.y));
            float dmin = std::min(dx, dy);
            float dmax = std::max(dx, dy);
            return (1.41421356f - 1.0f) * dmin + dmax;
        }

        // BFS reachability check. Resets has_visited on the way out so the grid
        // is clean for the actual planner.
        bool path_exists(coordi src, coordi dst)
        {
            auto [rows_sz, cols_sz] = this->grid->size();
            std::deque<coordi> q;
            std::vector<coordi> touched;
            q.push_back(src);
            (*this->grid)[src].has_visited = true;
            touched.push_back(src);

            bool found = false;
            while (!q.empty())
            {
                coordi curr = q.front();
                q.pop_front();
                if (curr == dst) { found = true; break; }

                for (coordi d : directions)
                {
                    coordi next = curr + d;
                    if (next.x < 0 || next.x >= (int32_t)cols_sz ||
                        next.y < 0 || next.y >= (int32_t)rows_sz)
                        continue;
                    if ((*this->grid)[next].has_visited || this->grid->is_obstacle(next))
                        continue;

                    (*this->grid)[next].has_visited = true;
                    touched.push_back(next);
                    q.push_back(next);
                }
            }

            // restore has_visited so we don't poison subsequent searches
            for (const coordi &c : touched)
                (*this->grid)[c].has_visited = false;

            return found;
        }

        /**
         * After a sensor update along the spline (or anywhere), bump the cost field
         * inside an ellipsoidal region defined by `cov` around `center`, then notify
         * D* Lite by calling update_vertex on each touched cell.
         */
        void update_grid(const Eigen::Vector2d &center,
                         const Eigen::Matrix2d &cov,
                         std::function<float(float, float)> compute_cost_at)
        {
            // axis-aligned bounding box that contains the 3-sigma ellipse.
            // Use eigenvalues so rotation is captured (cov(0,0)/(1,1) alone is not enough).
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(cov);
            float lam_max = static_cast<float>(es.eigenvalues().maxCoeff());
            int r_box = static_cast<int>(std::ceil(3.0f * std::sqrt(std::max(lam_max, 0.0f))));

            Eigen::Matrix2d sinv = cov.inverse();

            int icx = static_cast<int>(std::round(center[0]));
            int icy = static_cast<int>(std::round(center[1]));

            // collect the cells we touched so we only call update_vertex once each.
            std::vector<coordi> touched;
            touched.reserve((2 * r_box + 1) * (2 * r_box + 1));

            for (int dy = -r_box; dy <= r_box; dy++)
            {
                for (int dx = -r_box; dx <= r_box; dx++)
                {
                    int gx = icx + dx;
                    int gy = icy + dy;

                    // Correct bounds check: separate gx/gy and use >= against rows/cols.
                    if (gx < 0 || gy < 0 ||
                        gx >= (int)this->cols || gy >= (int)this->rows)
                        continue;

                    // Mahalanobis-distance gate: drop points outside the 3σ ellipse
                    Eigen::Vector2d r;
                    r << gx - center.x(), gy - center.y();
                    double m2 = r.transpose() * sinv * r;
                    if (m2 > 9.0)
                        continue;

                    float computed_cost = compute_cost_at(static_cast<float>(gx),
                                                          static_cast<float>(gy));

                    coordi c{gx, gy};
                    datapoint &dp = (*this->grid)[c];
                    dp.is_obstacle = true;
                    // We *accumulate the worst* spline-penalty seen at this cell.
                    // (Field is named dist_to_spline for legacy reasons; it stores cost.)
                    // dp.dist_to_spline = std::max(dp.dist_to_spline, computed_cost);
                    touched.push_back(c);
                }
            }

            for (const coordi &c : touched)
                this->update_vertex(c);
        }

        void dump_path_csv(const std::string &path_filename = "dstar_path.csv",
                           const std::string &cost_filename = "dstar_cost.csv")
        {
            auto [path_ptr, len] = this->extract_path();

            std::ofstream pf(path_filename);
            pf << "x,y\n";
            if (path_ptr != nullptr)
                for (uint32_t i = 0; i < len; i++)
                    pf << path_ptr[i].x << "," << path_ptr[i].y << "\n";

            std::ofstream cf(cost_filename);
            cf << "x,y,is_obstacle,dist_to_spline,g\n";
            for (uint32_t y = 0; y < this->rows; y++)
            {
                for (uint32_t x = 0; x < this->cols; x++)
                {
                    coordi c{(int32_t)x, (int32_t)y};
                    cf << x << "," << y << ","
                       << (this->grid->is_obstacle(c) ? 1 : 0) << ","
                       << (*this->grid)[c].dist_to_spline << ","
                       << (*this->grid)[c].g << "\n";
                }
            }
        }
    };
};
#endif