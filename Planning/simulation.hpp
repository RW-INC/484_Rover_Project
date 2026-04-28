#ifndef __SIMULATION_HPP__
#define __SIMULATION_HPP__

#include "Eigen/Dense"
#include "spline.h"
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
#include "../Nav/ekf.hpp"

#include <chrono>
#include <iomanip>

namespace simulation
{
    // wrap to [0, 2π]
    inline double wrap_2pi(double angle)
    {
        angle = fmod(angle, 2.0 * M_PI);
        if (angle < 0)
            angle += 2.0 * M_PI;
        return angle;
    }

    // wrap to [-π, π]
    inline double wrap_pi(double angle)
    {
        angle = fmod(angle + M_PI, 2.0 * M_PI);
        if (angle < 0)
            angle += 2.0 * M_PI;
        return angle - M_PI;
    }

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

        const state_eval &get_initial_state() const { return this->s0; }
        const state_eval &get_final_state() const { return this->sf; }

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
            std::uniform_real_distribution<double> dis_angle(0, M_PI);

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

            return std::max(1.2 * std::hypot(max_abs_x, max_abs_y), 5.0);
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
            std::uniform_real_distribution<double> phase_dist(M_PI / 2, M_PI);
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
                    (2.0 * M_PI * freq * this->axis.array() + phase_x).sin().matrix();
                const Eigen::VectorXd y_wave =
                    (2.0 * M_PI * freq * this->axis.array() + phase_y).cos().matrix();

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
            const double target_grad = std::tan(20.0 * M_PI / 180.0);

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

    Eigen::Vector3d LanderSunPointingVector(float_t t_lunar_day, float_t t_relative)
    {
        // --- 1. CONSTANTS ---
        double tau = 1.54;
        double Y_period = 365.25;
        double n = 172;
        double t0 = 0;
        double L = -90 * M_PI / 180; // South Pole
        double T_lunar = 708;
        double omega = 360 / T_lunar;
        double t_lunar_offset = (t_lunar_day - 2831) * 24;

        // --- 2. CALCULATION LOOP ---
        double ti = t_relative;
        double t_abs = ti + t_lunar_offset;

        // Solar declination (must be radians for trig below)
        double delta = (tau * sin(M_PI / 180.0 * (360.0 / Y_period) * (n + t_abs / 24.0 - t0))) * M_PI / 180.0;

        // Hour angle
        double H = M_PI / 180.0 * (omega * (T_lunar / 2 - t_abs));

        // Solar elevation
        double beta = asin(cos(L) * cos(delta) * cos(H) + sin(L) * sin(delta));

        // Solar Azimuth
        double phi_s = atan2(cos(delta) * sin(H), sin(L) * cos(delta) * cos(H) - cos(L) * sin(delta));

        // --- 3. GENERATE VECTORS ---
        // X = East, Y = North, Z = Up
        return Eigen::Vector3d(cos(beta) * sin(phi_s),
                               cos(beta) * cos(phi_s),
                               sin(beta));
    }

    Eigen::Matrix3d TRIAD(Eigen::Vector3d L_r_sun, Eigen::Vector3d R_r_sun, Eigen::Vector3d R_r_gravity)
    {
        // Rover LVLH
        auto R_t1 = R_r_gravity;
        auto R_t2 = R_t1.cross(R_r_sun).normalized();
        auto R_t3 = R_t1.cross(R_t2);

        // Lunar LVLH
        Eigen::Vector3d L_t1(0, 0, -1);
        Eigen::Vector3d L_t2 = L_t1.cross(L_r_sun).normalized();
        auto L_t3 = L_t1.cross(L_t2);

        Eigen::Matrix3d M_rover;
        M_rover.col(0) = R_t1;
        M_rover.col(1) = R_t2;
        M_rover.col(2) = R_t3;

        Eigen::Matrix3d M_lunar;
        M_lunar.col(0) = L_t1;
        M_lunar.col(1) = L_t2;
        M_lunar.col(2) = L_t3;

        return M_lunar * M_rover.transpose();
    }

    using ControllerFunction = std::function<Eigen::VectorXd(
        const Eigen::VectorXd &state,
        const Eigen::VectorXd &reference,
        const Eigen::VectorXd &control,
        double dt)>;

    using SimulationFunction = std::function<Eigen::VectorXd(
        const Eigen::VectorXd &state,
        const Eigen::VectorXd &control_input,
        const terrain &env,
        double dt)>;

    struct geom
    {
        double r = 0.17 / 2.0;
        double B = 0.2;
        double L = 0.25;
        double maxw = 1.0;
    };

#include <chrono>
    void run_simulation(ControllerFunction &controller, SimulationFunction &sim_func,
                        const trajectory &traj, const terrain &terr,
                        double dt, const std::string &output_file,
                        uint32_t control_decim, uint32_t estimation_decim, geom &rover_dims)
    {
        std::mt19937 gen(std::random_device{}());

        auto s0 = terr.get_projected_trajectory()[0];
        Eigen::Vector3d lander_pos(s0.x, s0.y, s0.z);

        double sigma_accel = 0.003;
        double sigma_gyro = 0.15 * M_PI / 180.0 / 60.0;
        double sigma_pos = 0.003;
        double sigma_vel = 0.0003;
        double sigma_rho = 0.001;
        double sigma_rhodot = 0.0001;
        double sigma_point = 0.001;
        double bias_rate_gyro = 0.5 * M_PI / 180.0 / 60.0;
        double yaw0 = 0.0;

        Eigen::Vector4d q0(cos(yaw0 / 2), 0, 0, sin(yaw0 / 2));
        Eigen::VectorXd mekf_state(7);
        mekf_state << q0, 0, 0, 0;

        Eigen::Matrix<double, 6, 6> mekf_P;
        mekf_P.setZero();
        mekf_P.diagonal() << 0.1, 0.1, 0.1, 0.001, 0.001, 0.001;

        Eigen::VectorXd trans_state(6);
        trans_state << 0.1, 0.1, 0.1, 0, 0, 0;

        Eigen::Matrix<double, 6, 6> trans_P;
        trans_P.setZero();
        trans_P.diagonal() << 0.1, 0.1, 0.1, 0.01, 0.01, 0.01;

        const auto reference_traj = terr.get_projected_trajectory();
        assert(!reference_traj.empty());
        std::cout << "Running simulation with " << reference_traj.size() << " steps..." << std::endl;

        Eigen::VectorXd state(9);
        // create the initial state
        state << lander_pos, 0, 0, 0, 0, 0, 0;
        Eigen::VectorXd control_input = Eigen::VectorXd::Zero(4);

        std::ofstream csv("full_state_output.csv");
        csv << std::setprecision(15) << std::scientific;

        csv << "t,x,y,z,vx,vy,vz,roll,pitch,yaw,"
            << "xe,ye,ze,vxe,vye,vze,roll_e,pitch_e,yaw_e,"
            << "wr,wl,dwr,dwl,"
            << "s1,s2,s3,"
            << "mu_r_est,mu_l_est,mu_r_act,mu_l_act\n";

        double yaw_est = 0, pitch_est = 0, roll_est = 0;
        Eigen::Vector3d pos_est = lander_pos;
        Eigen::Vector3d vel_est = Eigen::Vector3d::Zero();

        double mission_time = 0.0;
        double mu_r_est = 0.0;
        double mu_l_est = 0.0;
        double mu_r_act = 1.0;
        double mu_l_act = 1.0;

        for (size_t i = 0; i < reference_traj.size(); ++i)
        {
            mission_time += dt;

            const auto &ref = reference_traj[i];
            Eigen::VectorXd reference(6);
            reference << ref.x, ref.y, ref.vel_x, ref.vel_y, ref.theta, ref.theta_dot;

            // 1. Propagate true state
            auto sim_timing0 = std::chrono::high_resolution_clock::now();
            auto dstate = sim_func(state, control_input, terr, dt);
            auto sim_timing1 = std::chrono::high_resolution_clock::now();

            state += dstate * dt;

            // yaw wrap after state propagation
            state[8] = wrap_pi(state[8]);
            state[7] = wrap_pi(state[7]); // pitch stays [-π, π]
            state[6] = wrap_pi(state[6]);

            auto yaw_true = state[8];
            auto pitch_true = state[7];
            auto roll_true = state[6];

            // 4. Estimation

            auto estimation_timing0 = std::chrono::high_resolution_clock::now();
            if (i % estimation_decim == 0)
            {
                Eigen::Matrix3d R3;
                R3 << cos(yaw_true), -sin(yaw_true), 0,
                    sin(yaw_true), cos(yaw_true), 0,
                    0, 0, 1;
                Eigen::Matrix3d R2;
                R2 << cos(pitch_true), 0, sin(pitch_true),
                    0, 1, 0,
                    -sin(pitch_true), 0, cos(pitch_true);
                Eigen::Matrix3d R1;
                R1 << 1, 0, 0,
                    0, cos(roll_true), -sin(roll_true),
                    0, sin(roll_true), cos(roll_true);
                Eigen::Matrix3d R_true = R3 * R2 * R1;

                // 2. Simulate sensors
                Eigen::Vector3d a_true(dstate[3], dstate[4], dstate[5]);
                Eigen::Vector3d g_lunar(0, 0, -1.625);
                std::normal_distribution<double> accel_dist(0.0, sigma_accel);
                Eigen::Vector3d imu_noise(accel_dist(gen), accel_dist(gen), accel_dist(gen));
                auto a_imu = R_true.transpose() * (a_true - g_lunar) + imu_noise;

                Eigen::Vector3d w_true(dstate(6), dstate(7), dstate(8));
                auto p_true = w_true(0) - sin(pitch_true) * w_true(2);
                auto q_true = cos(roll_true) * w_true(1) + sin(roll_true) * cos(pitch_true) * w_true(2);
                auto r_true = -sin(roll_true) * w_true(1) + cos(roll_true) * cos(pitch_true) * w_true(2);
                Eigen::Vector3d w_body_true(p_true, q_true, r_true);

                std::normal_distribution<double> gyro_dist(0.0, sigma_gyro);
                Eigen::Vector3d gyro_noise(gyro_dist(gen), gyro_dist(gen), gyro_dist(gen));
                auto w_imu = w_body_true + Eigen::Vector3d::Constant(bias_rate_gyro * dt) + gyro_noise;

                Eigen::Vector3d pos_rel(state[0], state[1], state[2]);
                pos_rel -= lander_pos;
                Eigen::Vector3d vel_true_vec(state[3], state[4], state[5]);
                double rho_true = pos_rel.norm();
                double rhodot_true = pos_rel.dot(vel_true_vec) / std::max(rho_true, 1e-6);

                std::normal_distribution<double> rho_dist(0.0, sigma_rho);
                auto uwb_rho = rho_dist(gen) + rho_true;

                std::normal_distribution<double> rhodot_dist(0.0, sigma_rhodot);
                auto uwb_rhodot = rhodot_dist(gen) + rhodot_true;

                auto hours_elapsed = mission_time / (3600.0);
                Eigen::Vector3d s_dir = LanderSunPointingVector(2831, hours_elapsed);

                std::normal_distribution<double> pos_dist(0.0, sigma_pos);
                Eigen::Vector3d pos_noise(pos_dist(gen), pos_dist(gen), pos_dist(gen));
                std::normal_distribution<double> vel_dist(0.0, sigma_vel);
                Eigen::Vector3d vel_noise(vel_dist(gen), vel_dist(gen), vel_dist(gen));

                Eigen::Vector3d imu_pos = Eigen::Vector3d(state[0], state[1], state[2]) + pos_noise;
                Eigen::Vector3d imu_vel = Eigen::Vector3d(state[3], state[4], state[5]) + vel_noise;

                std::normal_distribution<double> sun_dist(0.0, sigma_point);
                Eigen::Vector3d sun_noise(sun_dist(gen), sun_dist(gen), sun_dist(gen));
                double angle = sun_noise.norm();
                Eigen::Quaterniond noisy_quaternion = (angle < 1e-10)
                                                          ? Eigen::Quaterniond::Identity()
                                                          : Eigen::Quaterniond(Eigen::AngleAxisd(angle, sun_noise / angle));

                auto g_body = -a_imu / a_imu.norm();
                Eigen::Vector3d R_r_sun_est = R_true.transpose() * s_dir;
                R_r_sun_est = R_r_sun_est / R_r_sun_est.norm();

                auto L_r_sun = s_dir / s_dir.norm();
                auto R_triad = TRIAD(L_r_sun, R_r_sun_est, g_body);
                Eigen::Quaterniond q_triad_obj(R_triad);
                q_triad_obj.normalize();

                Eigen::Matrix<double, 6, 6> mekf_P_new;
                Eigen::VectorXd mekf_state_new;
                rotational_mekf(q_triad_obj, w_imu, mekf_state,
                                mekf_P, dt * estimation_decim, sigma_gyro, mekf_P_new, mekf_state_new);
                mekf_P = mekf_P_new;

                mekf_state = mekf_state_new;
                Eigen::Quaterniond q_est(mekf_state[0], mekf_state[1], mekf_state[2], mekf_state[3]);
                Eigen::Matrix3d R_est_mat = q_est.toRotationMatrix();

                yaw_est = atan2(R_est_mat(1, 0), R_est_mat(0, 0));
                pitch_est = asin(-R_est_mat(2, 0));
                roll_est = atan2(R_est_mat(2, 1), R_est_mat(2, 2));

                yaw_est = wrap_pi(yaw_est);
                pitch_est = wrap_pi(pitch_est);
                roll_est = wrap_pi(roll_est);

                Eigen::Vector3d imu_pos_rel = imu_pos - lander_pos;
                Eigen::VectorXd imu_meas(6);
                imu_meas << imu_pos_rel, imu_vel;

                Eigen::Vector2d uwb_meas(uwb_rho, uwb_rhodot);
                Eigen::Vector3d orient_est(roll_est, pitch_est, yaw_est);

                Eigen::Matrix<double, 6, 6> trans_P_new;
                Eigen::VectorXd trans_state_new;
                translational_ekf(imu_meas, uwb_meas, orient_est, control_input,
                                  trans_state, trans_P, mekf_P, dt * estimation_decim, trans_P_new, trans_state_new);
                trans_P = trans_P_new;
                trans_state = trans_state_new;

                pos_est = trans_state.head<3>() + lander_pos;
                vel_est = trans_state.segment<3>(3);

                // estimate slip
                Eigen::Vector3d V_body_est = R_est_mat.transpose() * vel_est;
                double V_long_est = V_body_est[0]; // Longitudinal velocity

                // 2. Get the estimated body yaw rate (gyro z - estimated bias z)
                // mekf_state is [q0, q1, q2, q3, bx, by, bz], so bz is index 6
                double r_est_val = w_imu[2] - mekf_state[6];

                // 3. Calculate wheel velocities using differential kinematics
                double V_long_r_est = V_long_est + (rover_dims.B / 2.0) * r_est_val;
                double V_long_l_est = V_long_est - (rover_dims.B / 2.0) * r_est_val;

                // 4. Estimate mu (V_wheel / (r * omega))
                mu_r_est =  std::abs(control_input[0]) < 0.1 ? 1 : std::abs(V_long_r_est / (rover_dims.r * control_input[0]));
                mu_l_est = std::abs(control_input[1]) < 0.1 ? 1 : std::abs(V_long_l_est / (rover_dims.r * control_input[1]));

                //  Actual slip model
                mu_r_act = sin(10 * abs(control_input[0]));
                mu_l_act = sin(10 * abs(control_input[1]));


                if (pos_est.hasNaN() || vel_est.hasNaN())
                {
                    std::cerr << "NaN at step " << i << std::endl;
                    break;
                }
            }
            auto estimation_timing1 = std::chrono::high_resolution_clock::now();

            // 3. Controller
            auto controller_timing0 = std::chrono::high_resolution_clock::now();

            
            // 5. Tracking error
            double s1 = state[0] - ref.x;
            double s2 = state[1] - ref.y;
            double s3 = wrap_pi(state[8] - ref.theta);

            if (i % control_decim == 0)
            {
                Eigen::VectorXd est_state(9);
                est_state << pos_est[0], pos_est[1], pos_est[2],
                    vel_est[0], vel_est[1], vel_est[2],
                    roll_est, pitch_est, yaw_est;
                control_input = controller(est_state, reference, control_input, dt);
            }
            else
            {
                control_input[2] = 0.0;
                control_input[3] = 0.0;
            }
            auto controller_timing1 = std::chrono::high_resolution_clock::now();
            
            // 6. Write CSV
            csv << mission_time << ","
                << state[0] << "," << state[1] << "," << state[2] << ","
                << state[3] << "," << state[4] << "," << state[5] << ","
                << roll_true << "," << pitch_true << "," << yaw_true << ","
                << pos_est[0] << "," << pos_est[1] << "," << pos_est[2] << ","
                << vel_est[0] << "," << vel_est[1] << "," << vel_est[2] << ","
                << roll_est << "," << pitch_est << "," << yaw_est << ","
                << control_input[0] << "," << control_input[1] << ","
                << control_input[2] << "," << control_input[3] << ","
                << s1 << "," << s2 << "," << s3 << ","
                << mu_r_est << "," << mu_l_est << ","
                << mu_r_act << "," << mu_l_act << "\n";

            
            if (i % 1000 == 0)
            {
                std::cout << "Step " << i << " / " << reference_traj.size() << "          \n"
                          << "Simulation: " << std::chrono::duration<double>(sim_timing1 - sim_timing0).count() << "s          \n"
                          << "Estimation: " << std::chrono::duration<double>(estimation_timing1 - estimation_timing0).count() << "s          \n"
                          << "Controller: " << std::chrono::duration<double>(controller_timing1 - controller_timing0).count() << "s          "
                          << std::endl;
            }
        }
        csv.close();
        std::cout << "\nWrote full_state_output.csv" << std::endl;
    }
}

#endif
