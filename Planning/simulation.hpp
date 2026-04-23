#ifndef __SIMULATION_HPP__
#define __SIMULATION_HPP__

#include "Eigen/Dense"
#include "../spline/src/spline.h"
#include "../Planning/grid2d.hpp"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <functional>
#include <iostream>
#include <random>
#include <string>
#include <vector>

namespace simulation
{
    constexpr double k_pi = 3.14159265358979323846;

    using ControllerFunction = std::function<Eigen::VectorXd(
        const Eigen::VectorXd &state,
        const Eigen::VectorXd &reference,
        double dt)>;

    class trajectory
    {
    public:
        struct state_eval
        {
            double t;
            double x;
            double y;
            double vel_x;
            double vel_y;
            double theta;
            double theta_dot;
        };

    private:
        tk::spline x_spline;
        tk::spline y_spline;

        std::vector<double> t_knots;
        std::vector<double> x_knots;
        std::vector<double> y_knots;

        state_eval s0{};
        state_eval sf{};

    public:
        trajectory(double x0, double y0,
                   double min_vel, double max_vel, double min_dt, double max_dt, uint32_t num_points)
        {
            generate_trajectory(
                this->t_knots,
                this->x_knots,
                this->y_knots,
                min_dt,
                max_dt,
                x0,
                y0,
                min_vel,
                max_vel,
                num_points);

            this->x_spline.set_points(this->t_knots, this->x_knots);
            this->y_spline.set_points(this->t_knots, this->y_knots);

            this->s0 = evaluate_at_x(this->t_knots.front());
            this->sf = evaluate_at_x(this->t_knots.back());
        }

        const std::vector<double> &get_x_knots() const { return this->x_knots; }
        const std::vector<double> &get_y_knots() const { return this->y_knots; }

        std::vector<state_eval> sample(double dt) const
        {
            assert(dt > 0.0);
            std::vector<state_eval> states;
            states.reserve(static_cast<size_t>((this->sf.t - this->s0.t) / dt) + 2);

            for (double t = this->s0.t; t < this->sf.t; t += dt)
            {
                states.push_back(evaluate_at_x(t));
            }

            states.push_back(this->sf);
            return states;
        }

        state_eval evaluate_at_x(double time) const
        {
            state_eval eval{};

            eval.t = time;
            eval.x = this->x_spline(time);
            eval.y = this->y_spline(time);
            eval.vel_x = this->x_spline.deriv(1, time);
            eval.vel_y = this->y_spline.deriv(1, time);
            eval.theta = atan2(eval.vel_y, eval.vel_x);
            eval.theta_dot =
                (eval.vel_x * this->y_spline.deriv(2, time) -
                 eval.vel_y * this->x_spline.deriv(2, time)) /
                (eval.vel_x * eval.vel_x + eval.vel_y * eval.vel_y);

            return eval;
        }

        static void generate_trajectory(
            std::vector<double> &t_out,
            std::vector<double> &x_out,
            std::vector<double> &y_out,
            double min_dt,
            double max_dt,
            double x0,
            double y0,
            double min_vel,
            double max_vel,
            uint32_t num_points)
        {
            assert(num_points >= 3);
            std::random_device rd;
            std::mt19937 gen(rd());

            std::uniform_real_distribution<double> dis_vel(min_vel, max_vel);
            std::uniform_real_distribution<double> dis_dt(min_dt, max_dt);
            std::uniform_real_distribution<double> dis_angle(-k_pi, k_pi);

            const uint32_t n = num_points;
            t_out.resize(n);
            x_out.resize(n);
            y_out.resize(n);

            t_out[0] = 0.0;
            x_out[0] = x0;
            y_out[0] = y0;

            for (uint32_t i = 1; i < n; i++)
            {
                const double dt = dis_dt(gen);
                const double v = dis_vel(gen);
                const double r = v * dt;
                const double angle = dis_angle(gen);

                x_out[i] = x_out[i - 1] + r * cos(angle);
                y_out[i] = y_out[i - 1] + r * sin(angle);
                t_out[i] = t_out[i - 1] + dt;
            }
        }
    };

    class terrain
    {
    public:
        struct traj_on_surface
        {
            double t;
            double x;
            double y;
            double z;
            double vel_x;
            double vel_y;
            double theta;
            double theta_dot;
        };

    private:
        Eigen::VectorXd axis;
        Eigen::MatrixXd z_map;
        double patch_size = 1.0;
        double hover_height = 0.0;
        std::vector<traj_on_surface> projected_traj;

    public:
        terrain(uint32_t resolution, double patch_size)
            : patch_size(patch_size),
              hover_height(patch_size * 0.005)
        {
            generate_surface(resolution);
        }

        terrain(uint32_t resolution, const trajectory &traj, double dt = 0.01)
            : patch_size(compute_patch_size(traj))
        {
            this->hover_height = this->patch_size * 0.005;
            generate_surface(resolution);
            store_projected_trajectory(traj, dt);
        }

        double interp2(double x, double y) const
        {
            return bilinear_sample(this->z_map, this->axis, this->axis, x, y);
        }

        const std::vector<traj_on_surface> &get_projected_trajectory() const { return this->projected_traj; }

        void write_to_file(const std::string &filename) const
        {
            std::ofstream file(filename, std::ios::binary);
            file.exceptions(std::ios::failbit | std::ios::badbit);

            const char magic[8] = {'S', 'P', 'X', 'T', 'E', 'R', '1', '\0'};
            const uint32_t rows = static_cast<uint32_t>(this->z_map.rows());
            const uint32_t cols = static_cast<uint32_t>(this->z_map.cols());
            const double x_min = this->axis[0];
            const double x_max = this->axis[this->axis.size() - 1];
            const double y_min = x_min;
            const double y_max = x_max;

            using RowMajorMatrixXd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
            const RowMajorMatrixXd z_row_major = this->z_map;

            file.write(magic, sizeof(magic));
            file.write(reinterpret_cast<const char *>(&rows), sizeof(rows));
            file.write(reinterpret_cast<const char *>(&cols), sizeof(cols));
            file.write(reinterpret_cast<const char *>(&x_min), sizeof(x_min));
            file.write(reinterpret_cast<const char *>(&x_max), sizeof(x_max));
            file.write(reinterpret_cast<const char *>(&y_min), sizeof(y_min));
            file.write(reinterpret_cast<const char *>(&y_max), sizeof(y_max));
            file.write(
                reinterpret_cast<const char *>(z_row_major.data()),
                static_cast<std::streamsize>(sizeof(double) * rows * cols));
        }

        void write_projected_trajectory_to_file(const std::string &filename) const
        {
            std::ofstream file(filename, std::ios::binary);
            file.exceptions(std::ios::failbit | std::ios::badbit);

            const char magic[8] = {'S', 'P', 'X', 'T', 'R', 'J', '1', '\0'};
            const uint32_t count = static_cast<uint32_t>(this->projected_traj.size());

            file.write(magic, sizeof(magic));
            file.write(reinterpret_cast<const char *>(&count), sizeof(count));
            for (const auto &state : this->projected_traj)
            {
                const double values[8] = {
                    state.t,
                    state.x,
                    state.y,
                    state.z,
                    state.vel_x,
                    state.vel_y,
                    state.theta,
                    state.theta_dot,
                };
                file.write(reinterpret_cast<const char *>(values), sizeof(values));
            }
        }

    private:
        static double compute_patch_size(const trajectory &traj)
        {
            double max_abs_x = 0.0;
            double max_abs_y = 0.0;

            for (double x : traj.get_x_knots())
            {
                max_abs_x = std::max(max_abs_x, std::abs(x));
            }
            for (double y : traj.get_y_knots())
            {
                max_abs_y = std::max(max_abs_y, std::abs(y));
            }

            return std::max(2.0 * std::hypot(max_abs_x, max_abs_y), 5.0);
        }

        void store_projected_trajectory(const trajectory &traj, double dt)
        {
            assert(dt > 0.0);
            this->projected_traj.clear();
            const auto states = traj.sample(dt);
            this->projected_traj.reserve(states.size());

            for (const auto &state : states)
            {
                this->projected_traj.push_back({
                    state.t,
                    state.x,
                    state.y,
                    interp2(state.x, state.y) + this->hover_height,
                    state.vel_x,
                    state.vel_y,
                    state.theta,
                    state.theta_dot,
                });
            }
        }

        void generate_surface(uint32_t resolution)
        {
            assert(resolution >= 3);
            const int n = static_cast<int>(resolution);
            this->axis = Eigen::VectorXd::LinSpaced(
                n,
                -this->patch_size,
                this->patch_size);
            this->z_map = Eigen::MatrixXd::Zero(n, n);

            const double wobble_amp = this->patch_size * 0.15;
            const double terrain_amp = this->patch_size * 0.03;
            constexpr int num_craters = 80;
            const double dx = (this->axis[n - 1] - this->axis[0]) /
                              static_cast<double>(n - 1);

            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_real_distribution<double> phase_dist(0.0, 2.0 * k_pi);
            std::uniform_real_distribution<double> crater_center_dist(-this->patch_size, this->patch_size);
            std::uniform_real_distribution<double> crater_radius_dist(
                this->patch_size * 0.01,
                this->patch_size * 0.06);
            std::uniform_real_distribution<double> crater_depth_dist(
                this->patch_size * 0.002,
                this->patch_size * 0.012);

            this->z_map += generate_wobble_surface(this->axis, this->patch_size, wobble_amp, gen);

            for (int harmonic = 1; harmonic <= 5; ++harmonic)
            {
                const double freq = (harmonic * 0.1) / this->patch_size;
                const double amp = terrain_amp / harmonic;
                const double phase_x = phase_dist(gen);
                const double phase_y = phase_dist(gen);
                const Eigen::VectorXd x_wave =
                    (2.0 * k_pi * freq * this->axis.array() + phase_x).sin().matrix();
                const Eigen::VectorXd y_wave =
                    (2.0 * k_pi * freq * this->axis.array() + phase_y).cos().matrix();

                this->z_map.noalias() += amp * (y_wave * x_wave.transpose());
            }

            for (int crater = 0; crater < num_craters; ++crater)
            {
                const double center_x = crater_center_dist(gen);
                const double center_y = crater_center_dist(gen);
                const double radius = crater_radius_dist(gen);
                const double depth = crater_depth_dist(gen);
                const double influence_radius = 4.0 * radius;
                const int col_min = std::max(
                    0,
                    static_cast<int>(std::floor((center_x - influence_radius - this->axis[0]) / dx)));
                const int col_max = std::min(
                    n - 1,
                    static_cast<int>(std::ceil((center_x + influence_radius - this->axis[0]) / dx)));
                const int row_min = std::max(
                    0,
                    static_cast<int>(std::floor((center_y - influence_radius - this->axis[0]) / dx)));
                const int row_max = std::min(
                    n - 1,
                    static_cast<int>(std::ceil((center_y + influence_radius - this->axis[0]) / dx)));

                const int local_cols = col_max - col_min + 1;
                const int local_rows = row_max - row_min + 1;

                const Eigen::ArrayXd x_local =
                    this->axis.segment(col_min, local_cols).array() - center_x;
                const Eigen::ArrayXd y_local =
                    this->axis.segment(row_min, local_rows).array() - center_y;

                const Eigen::ArrayXXd dist_sq =
                    y_local.square().matrix().replicate(1, local_cols).array() +
                    x_local.square().transpose().matrix().replicate(local_rows, 1).array();

                const double crater_sigma_sq = radius * radius;
                const double bowl_sigma_sq = (radius * 0.7) * (radius * 0.7);

                this->z_map.block(row_min, col_min, local_rows, local_cols).array() +=
                    -depth * (-dist_sq / (2.0 * bowl_sigma_sq)).exp() +
                    0.25 * depth * (-dist_sq / (2.0 * crater_sigma_sq)).exp();
            }

            const int coarse_blur_radius = std::max<int>(4, static_cast<int>(resolution / 50));
            const double coarse_blur_sigma = std::max(1.75, 0.55 * coarse_blur_radius);
            const int polish_blur_radius = std::max(2, coarse_blur_radius / 2);
            const double polish_blur_sigma = std::max(1.0, 0.6 * polish_blur_radius);

            // A broader pass removes the blockier crater/wobble transitions,
            // then a lighter pass polishes the remaining pixel-scale roughness.
            this->z_map = gaussian_convolve(this->z_map, coarse_blur_radius, coarse_blur_sigma);
            this->z_map = gaussian_convolve(this->z_map, polish_blur_radius, polish_blur_sigma);

            const double max_grad = compute_max_gradient(this->z_map, dx);
            const double target_grad = std::tan(55.0 * k_pi / 180.0);

            if (max_grad > target_grad)
            {
                this->z_map *= target_grad / max_grad;
            }
        }

        static double bilinear_sample(
            const Eigen::MatrixXd &values,
            const Eigen::VectorXd &x_axis,
            const Eigen::VectorXd &y_axis,
            double x,
            double y)
        {
            const int cols = static_cast<int>(values.cols());
            const int rows = static_cast<int>(values.rows());
            const double dx = x_axis[1] - x_axis[0];
            const double dy = y_axis[1] - y_axis[0];

            const double fx = (x - x_axis[0]) / dx;
            const double fy = (y - y_axis[0]) / dy;

            const int ix = std::clamp(static_cast<int>(std::floor(fx)), 0, cols - 2);
            const int iy = std::clamp(static_cast<int>(std::floor(fy)), 0, rows - 2);

            const double tx = std::clamp(fx - ix, 0.0, 1.0);
            const double ty = std::clamp(fy - iy, 0.0, 1.0);

            const double z00 = values(iy, ix);
            const double z10 = values(iy, ix + 1);
            const double z01 = values(iy + 1, ix);
            const double z11 = values(iy + 1, ix + 1);

            return (1.0 - tx) * (1.0 - ty) * z00 +
                   tx * (1.0 - ty) * z10 +
                   (1.0 - tx) * ty * z01 +
                   tx * ty * z11;
        }

        static Eigen::VectorXd make_gaussian_kernel(int radius, double sigma)
        {
            const int size = 2 * radius + 1;
            Eigen::VectorXd kernel(size);
            double sum = 0.0;

            for (int i = -radius; i <= radius; ++i)
            {
                const double value = std::exp(-(i * i) / (2.0 * sigma * sigma));
                kernel[i + radius] = value;
                sum += value;
            }

            kernel /= sum;
            return kernel;
        }

        static Eigen::MatrixXd gaussian_convolve(const Eigen::MatrixXd &input, int radius, double sigma)
        {
            const Eigen::VectorXd kernel = make_gaussian_kernel(radius, sigma);
            Eigen::MatrixXd temp = Eigen::MatrixXd::Zero(input.rows(), input.cols());
            Eigen::MatrixXd output = Eigen::MatrixXd::Zero(input.rows(), input.cols());

            for (int row = 0; row < input.rows(); ++row)
            {
                for (int col = 0; col < input.cols(); ++col)
                {
                    double sum = 0.0;
                    for (int k = -radius; k <= radius; ++k)
                    {
                        const int sample_col = std::clamp(col + k, 0, static_cast<int>(input.cols()) - 1);
                        sum += kernel[k + radius] * input(row, sample_col);
                    }
                    temp(row, col) = sum;
                }
            }

            for (int row = 0; row < input.rows(); ++row)
            {
                for (int col = 0; col < input.cols(); ++col)
                {
                    double sum = 0.0;
                    for (int k = -radius; k <= radius; ++k)
                    {
                        const int sample_row = std::clamp(row + k, 0, static_cast<int>(input.rows()) - 1);
                        sum += kernel[k + radius] * temp(sample_row, col);
                    }
                    output(row, col) = sum;
                }
            }

            return output;
        }

        static Eigen::MatrixXd generate_wobble_surface(
            const Eigen::VectorXd &axis,
            double patch_size,
            double wobble_amp,
            std::mt19937 &gen)
        {
            constexpr int wobble_nodes = 9;
            Eigen::VectorXd node_axis = Eigen::VectorXd::LinSpaced(wobble_nodes, -patch_size, patch_size);
            Eigen::MatrixXd coarse_noise(wobble_nodes, wobble_nodes);
            std::normal_distribution<double> normal_dist(0.0, 1.0);

            for (int row = 0; row < wobble_nodes; ++row)
            {
                for (int col = 0; col < wobble_nodes; ++col)
                {
                    coarse_noise(row, col) = normal_dist(gen);
                }
            }

            Eigen::MatrixXd wobble(axis.size(), axis.size());
            for (int row = 0; row < axis.size(); ++row)
            {
                for (int col = 0; col < axis.size(); ++col)
                {
                    wobble(row, col) = bilinear_sample(
                        coarse_noise,
                        node_axis,
                        node_axis,
                        axis[col],
                        axis[row]);
                }
            }

            const double max_abs = wobble.cwiseAbs().maxCoeff();
            wobble *= wobble_amp / max_abs;

            const int wobble_blur_radius = std::max<int>(2, static_cast<int>(axis.size() / 75));
            const double wobble_blur_sigma = std::max(1.2, 0.5 * wobble_blur_radius);
            wobble = gaussian_convolve(wobble, wobble_blur_radius, wobble_blur_sigma);

            return wobble;
        }

        static double compute_max_gradient(const Eigen::MatrixXd &surface, double dx)
        {
            double max_grad = 0.0;

            for (int row = 0; row < surface.rows(); ++row)
            {
                const int row_lo = std::max(row - 1, 0);
                const int row_hi = std::min(row + 1, static_cast<int>(surface.rows()) - 1);

                for (int col = 0; col < surface.cols(); ++col)
                {
                    const int col_lo = std::max(col - 1, 0);
                    const int col_hi = std::min(col + 1, static_cast<int>(surface.cols()) - 1);

                    const double dzdx = (surface(row, col_hi) - surface(row, col_lo)) /
                                        ((col_hi - col_lo) * dx);
                    const double dzdy = (surface(row_hi, col) - surface(row_lo, col)) /
                                        ((row_hi - row_lo) * dx);

                    max_grad = std::max(max_grad, std::hypot(dzdx, dzdy));
                }
            }

            return max_grad;
        }
    };
}

#endif
