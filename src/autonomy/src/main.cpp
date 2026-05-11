#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <string>
#include <array>
#include <vector>
#include <sstream>
#include <limits>
#include <fstream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "planning.hpp"
#include "Eigen/Dense"
#include "d_star.hpp"
#include "grid2d.hpp"
#include "lightpq.hpp"
#include "spline_testcase.hpp"
#include "spline.h"
#include "trajectories.hpp"

/**
 * Follow the spline, but use D* Lite to avoid obstacles.
 *
 *  0) user draws the spline (this is our reference trajectory)
 *  1) lidar tick → update grid2d
 *  2) check if current path is obstructed
 *  3) recompute via D* if so
 *  4) execute next reference, repeat
 */
class AutoNode : public rclcpp::Node
{
private:
    bool has_trajectory = false;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr spliner_vals;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr nav_estimations;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr auto_next_state;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<planning_module::NominalTrajectory> nt;

    std::shared_ptr<planning_module::grid2d> grid;
    std::unique_ptr<planning_module::d_star> planner;
    std::shared_ptr<planning_module::Trajectory> active_trajectory;

    static constexpr uint32_t GRID_R = 300;
    static constexpr uint32_t GRID_C = 300;
    static constexpr double cells_per_meter = 50.0;

    


    void switch_trajectory(std::shared_ptr<planning_module::Trajectory> next)
    {
        if (active_trajectory == next) {
            return;
        }
        // else if (active_trajectory == nt) {
        //     this->curr_time = 0.0;
        // }
        // if (next == nt && active_trajectory != nullptr) {
        //     this->curr_time = this->final_time;
        //     this->final_time = 1.0;
        // }
        
        std::atomic_store(&active_trajectory, std::move(next));
    }

    std::optional<Eigen::VectorXd> get_reference(double t) const
    {
        auto traj = std::atomic_load(&active_trajectory);
        if (!traj)
            return std::nullopt;
        return traj->evaluate_at(t);
    }

    void recompute_cb(nav_msgs::msg::Odometry odom)
    {
        
        // std::cout << this->curr_time << std::endl;
        if (!this->has_trajectory)
            return;
        


        // Eigen::Vector3d lidar{0, 0, 0};

        // this should only be called when the lidar thinks there's an obstacle
        // lidar data is of the form (xs, ys, covs) with xs, ys, covs being "lists"
        // i've put this here as a vector3d; it will have to be a custon struct
        // in this for loop, then:

        // from my ACTIVE trajectory, find the first point on the nominal trajectory to go to.
        // how? linesearch using the time and lidar data, store the location of this first viable point (and time).

        // take the current point, target point. run d*
        // spline d*
        // that is the new active trajectory. repeat.
        // if i run out of active trajectory points, switch to nominal. if no more nominal DONE.
        auto traj = std::atomic_load(&this->active_trajectory);
        if (!traj)
            return;
        // TODO fix it
        Eigen::Vector2d center = Eigen::Vector2d::Zero();
        Eigen::Matrix2d cov = Eigen::Matrix2d::Identity();

        std::optional<Eigen::Vector2d> time_opt =
            this->nt->linesearch(center, cov, 1.0);
        if (!time_opt.has_value() || time_opt.value()[1] == 0)
        {
            return;
        }
        // we have to replan now...
        const double t_current = time_opt.value()[0];
        const double t_target = time_opt.value()[1];

        auto current = this->nt->evaluate_at(t_current);
        auto target = this->nt->evaluate_at(t_target);
        if (!current.has_value() || !target.has_value())
        {
            switch_trajectory(this->nt);
            return;
        }
        planning_module::coordi start_cell{
            (int32_t)std::round(current.value()[0]),
            (int32_t)std::round(current.value()[1])};
        planning_module::coordi goal_cell{
            (int32_t)std::round(target.value()[0]),
            (int32_t)std::round(target.value()[1])};

        this->planner = std::make_unique<planning_module::d_star>(
            GRID_R, GRID_C, start_cell, goal_cell,
            planning_module::coordi{0, 0}, this->grid);

        this->planner->compute_shortest_path();
        auto [path, length] = planner->extract_path();
        if (path == nullptr || length < 2)
        {
            return;
        }
        std::vector<double> t_knots, x_knots, y_knots;

        const double dt_total = t_target - t_current;
        for (uint32_t i = 0; i < length; i++)
        {
            t_knots.push_back(t_current + (dt_total * i / (length - 1)));
            x_knots.push_back(static_cast<double>(path[i].x));
            y_knots.push_back(static_cast<double>(path[i].y));
        }

        tk::spline x_spline, y_spline;
        x_spline.set_points(t_knots, x_knots);
        y_spline.set_points(t_knots, y_knots);

        auto deviation = std::make_shared<planning_module::ReplannedTrajectory>(
            std::move(x_spline), std::move(y_spline), t_current, t_target);
            
        switch_trajectory(deviation);
        // this->final_time = t_target;
    }

    void
    parse_testcase_cb(std_msgs::msg::String str)
    {
        if (this->has_trajectory)
            return;
        this->has_trajectory = true;

        spline_testcase sp;
        sp.parse_testcase(str.data);

        std::vector<float> elevation(GRID_R * GRID_C, 0.0f);
        std::vector<float> illumination(GRID_R * GRID_C, 0.0f);
        std::unique_ptr<bool[]> obstacles(new bool[GRID_R * GRID_C]());

        this->grid = std::make_shared<planning_module::grid2d>(
            GRID_R, GRID_C,
            elevation.data(), illumination.data(), obstacles.get());

        // create nominal trajectory with the sp val
        this->nt = std::make_shared<planning_module::NominalTrajectory>(std::move(sp));
        // this->curr_time = 0.0;
        // this->final_time = 1.0;
        std::atomic_store(&this->active_trajectory, std::static_pointer_cast<planning_module::Trajectory>(this->nt));
    }

    void publish_reference_cb()
    {
        // std::cout << this->has_trajectory << std::endl;
        if (!this->has_trajectory){
            nav_msgs::msg::Odometry odom;
            odom.pose.pose.orientation.x = 1.0;
            this->auto_next_state->publish(odom);
            return;
        }
        if (this->active_trajectory->curr_time() >= 1) {
            this->has_trajectory = false;
            return;
        }
        if (this->active_trajectory != this->nt && this->active_trajectory->curr_time() >= 1.0) {
            switch_trajectory(this->nt);
        }
        auto vec = this->active_trajectory->evaluate_at(this->active_trajectory->curr_time());

        nav_msgs::msg::Odometry odom;
        odom.pose.pose.orientation.x = 0.0;
        auto &pose = odom.pose.pose.position;
        pose.x = vec.value()[0];
        pose.y = vec.value()[1];
        pose.z = vec.value()[2]; // YAW
        auto &twist = odom.twist.twist.linear;
        twist.x = vec.value()[3];
        twist.y = vec.value()[4];
        twist.z = vec.value()[5];
        this->auto_next_state->publish(odom);
        this->active_trajectory->add_time();
    }

public:
    AutoNode() : Node("auto_node")
    {
        this->spliner_vals = this->create_subscription<std_msgs::msg::String>(
            "/spline_path", 10,
            std::bind(&AutoNode::parse_testcase_cb, this, std::placeholders::_1));

        this->nav_estimations = this->create_subscription<nav_msgs::msg::Odometry>(
            "/trans_est", 10, // change this to lidar topic
            std::bind(&AutoNode::recompute_cb, this, std::placeholders::_1));

        // need to make a callback that determibntes the publishing rate of the soline s.t. the smc only fires when we want it to
        this->auto_next_state = this->create_publisher<nav_msgs::msg::Odometry>(
            "/auto_next_state", 10);
        
        this->timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&AutoNode::publish_reference_cb, this)
            );
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutoNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}