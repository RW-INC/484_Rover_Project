#include <iostream>

#include "../Planning/simulation.hpp"
#include "Eigen/Dense"
#include <iostream>
#include <thread>
#include <chrono>

#include "../Nav/ekf.hpp"
#include <iostream>
#include <fstream>
#include <random>
#include "Eigen/Dense"
#include "../Nav/ekf.hpp"

int main()
{
    constexpr uint32_t terrain_resolution = 300;
    constexpr double trajectory_dt = 1e-3;

    simulation::trajectory traj(
        0.0, 0.0,
        0.01, 0.02,
        40.0, 70.00, 10);

    simulation::terrain lunar_surface(terrain_resolution, traj, trajectory_dt);

    lunar_surface.write_to_file("terrain.bin");
    lunar_surface.write_projected_trajectory_to_file("terrain_traj.bin");

    std::cout << "Wrote terrain.bin and terrain_traj.bin." << std::endl;
    std::cout << "Plot with: python Watchdog/visualize.py terrain.bin" << std::endl;
        
    // smc controller
    auto K = Eigen::Vector3d(0.1, 0.1, 2.0);
    auto eps = Eigen::Vector3d(5, 5, 5);
    simulation::geom rover_dims;

    // SMC using ControllerFunction
    simulation::ControllerFunction SMC = [K, eps, rover_dims](
                                             const Eigen::VectorXd &state, const Eigen::VectorXd &reference, const Eigen::VectorXd &control_input, double dt) -> Eigen::VectorXd
    {
        /**
         * Jacobian of H:
[(mu_R_sym*r_w_sym*cos(theta_sym))/2, (mu_L_sym*r_w_sym*cos(theta_sym))/2]
[(mu_R_sym*r_w_sym*sin(theta_sym))/2, (mu_L_sym*r_w_sym*sin(theta_sym))/2]
[     (mu_R_sym*r_w_sym)/B_track_sym,     -(mu_L_sym*r_w_sym)/B_track_sym]
         */

        /**
         * H:
         *
(r_w_sym*cos(theta_sym)*(mu_L_sym*w_l_sym + mu_R_sym*w_r_sym))/2
(r_w_sym*sin(theta_sym)*(mu_L_sym*w_l_sym + mu_R_sym*w_r_sym))/2
-(r_w_sym*(mu_L_sym*w_l_sym - mu_R_sym*w_r_sym))/B_track_sym
         *
         */

        // get the current state
        // x y vel_x vel_y theta thetadot

        auto ref_Xd = reference[2];
        auto ref_Yd = reference[3];
        auto ref_Thd = reference[5];

        // x,y,z velx, vely, velz, roll,pitch,yaw
        auto s_1 = state[0] - reference[0];
        auto s_2 = state[1] - reference[1];
        auto s_3 = simulation::wrap_pi(state[8] - reference[4]);
        auto s = Eigen::Vector3d(s_1, s_2, s_3);

        double rw = rover_dims.r;
        double B = rover_dims.B;
        auto mu_r = 1.0, mu_l = 1.0;

        auto theta = state[8];
        auto w_r = control_input[0];
        auto w_l = control_input[1];

        Eigen::Vector3d H;
        H[0] = (rw * cos(theta) * (mu_l * w_l + mu_r * w_r)) / 2;
        H[1] = (rw * sin(theta) * (mu_l * w_l + mu_r * w_r)) / 2;
        H[2] = -(rw * (mu_l * w_l - mu_r * w_r)) / B;

        Eigen::Matrix<double, 3, 2> Jacobian_H;
        Jacobian_H << (mu_r * rw * cos(theta)) / 2.0, (mu_l * rw * cos(theta)) / 2.0,
            (mu_r * rw * sin(theta)) / 2.0, (mu_l * rw * sin(theta)) / 2.0,
            (mu_r * rw) / B, -(mu_l * rw) / B;

        Eigen::Vector3d sigmoid_s = (s.array() / (2.0 * eps).array()).tanh().matrix();

        auto ref_dot = Eigen::Vector3d(ref_Xd, ref_Yd, ref_Thd);
        auto delta_u = Jacobian_H.completeOrthogonalDecomposition().pseudoInverse() * (ref_dot - H - K.asDiagonal() * sigmoid_s);

        Eigen::VectorXd output(4);
        output << w_r + delta_u[0], w_l + delta_u[1], delta_u[0], delta_u[1];
        output[0] = std::clamp(w_r + delta_u[0], -rover_dims.maxw, rover_dims.maxw);
        output[1] = std::clamp(w_l + delta_u[1], -rover_dims.maxw, rover_dims.maxw);

        return output;
    };

    simulation::SimulationFunction sim_func = [rover_dims](const Eigen::VectorXd &state, const Eigen::VectorXd &control_input, const simulation::terrain &env, double dt) -> Eigen::VectorXd
    {
        // unwrap geometry
        double r = rover_dims.r;
        double B = rover_dims.B;
        auto L = rover_dims.L;

        // x,y,z velx, vely, velz, roll,pitch,yaw
        auto yaw = state[8];
        auto pitch = state[7];

        auto w_r = control_input[0];  // new w_r
        auto w_l = control_input[1];  // new w_l
        auto dw_r = control_input[2]; // dw_r
        auto dw_l = control_input[3]; // dw_l

        auto mu_r = sin(10 * abs(w_r));
        auto mu_l = sin(10 * abs(w_l));

        // --- diff-drive kinematics ---
        auto v = (r / 2.0) * (mu_r * w_r + mu_l * w_l);

        // Velocities
        auto vx = v * cos(yaw) * cos(pitch);
        auto vy = v * sin(yaw) * cos(pitch);
        auto vz = v * sin(pitch);

        // Accelerations
        auto dv = (r / 2.0) * (mu_r * dw_r + mu_l * dw_l) / (dt);

        auto body_wheels = Eigen::Matrix<double, 2, 4>();
        body_wheels << L / 2.0, L / 2.0, -L / 2.0, -L / 2.0,
            -B / 2.0, B / 2.0, -B / 2.0, B / 2.0;

        // --- attitude rates (spatial perturbation) ---
        double eps_p = 1e-6;
        auto att = [&env, body_wheels](double xq, double yq, double yq_yaw) -> std::pair<double, double>
        {
            Eigen::Matrix2d Rq;
            Rq << cos(yq_yaw), -sin(yq_yaw),
                sin(yq_yaw), cos(yq_yaw);
            auto wq = Rq * body_wheels + Eigen::Vector2d(xq, yq).replicate(1, 4);
            Eigen::Vector4d zq;
            for (int i = 0; i < 4; ++i)
            {
                zq[i] = env.interp2(wq(0, i), wq(1, i));
            }
            Eigen::MatrixXd Aq(4, 3);
            Aq << wq.transpose(), Eigen::Vector4d::Ones();
            Eigen::Vector3d abcq = (Aq.transpose() * Aq).ldlt().solve(Aq.transpose() * zq);
            double ab = cos(yq_yaw) * abcq[0] + sin(yq_yaw) * abcq[1];
            double bb = -sin(yq_yaw) * abcq[0] + cos(yq_yaw) * abcq[1];
            double pq = atan(ab);
            double rq = atan(bb);
            return {rq, pq};
        };

        auto dyaw = (r / B) * (mu_r * w_r - mu_l * w_l);
        auto droll = (att(state[0] + eps_p, state[1], yaw).first - att(state[0] - eps_p, state[1], yaw).first) / (2.0 * eps_p) * vx +
                     (att(state[0], state[1] + eps_p, yaw).first - att(state[0], state[1] - eps_p, yaw).first) / (2.0 * eps_p) * vy +
                     (att(state[0], state[1], yaw + eps_p).first - att(state[0], state[1], yaw - eps_p).first) / (2.0 * eps_p) * dyaw;
        auto dpitch = (att(state[0] + eps_p, state[1], yaw).second - att(state[0] - eps_p, state[1], yaw).second) / (2.0 * eps_p) * vx +
                      (att(state[0], state[1] + eps_p, yaw).second - att(state[0], state[1] - eps_p, yaw).second) / (2.0 * eps_p) * vy +
                      (att(state[0], state[1], yaw + eps_p).second - att(state[0], state[1], yaw - eps_p).second) / (2.0 * eps_p) * dyaw;

        // --- accelerations (u constant within step) ---
        auto ax = cos(yaw) * cos(pitch) * dv - v * cos(pitch) * sin(yaw) * dyaw - v * cos(yaw) * sin(pitch) * dpitch;
        auto ay = sin(yaw) * cos(pitch) * dv + v * cos(pitch) * cos(yaw) * dyaw - v * sin(yaw) * sin(pitch) * dpitch;
        auto az = sin(pitch) * dv + v * cos(pitch) * dpitch;

        return (Eigen::VectorXd(9) << vx, vy, vz, ax, ay, az, droll, dpitch, dyaw).finished();
    };

    std::cout << "Running simulation..." << std::endl;

    simulation::run_simulation(SMC, sim_func, traj,
                               lunar_surface, trajectory_dt,
                               "simulation_output.bin", 5, 2, rover_dims);

    return 0;
}
