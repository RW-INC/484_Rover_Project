#include <Eigen/Dense>
#include <optional>
#include "spline.h"
#include "spline_testcase.hpp"


namespace planning_module {

    class Trajectory 
    {
    public:
        static constexpr double dt = 0.002;

        inline static auto mahalanobis(const Eigen::Matrix2d &cov, const Eigen::Vector2d &center, const Eigen::Vector2d &r) {return (r - center).transpose() * cov.inverse() * (r - center);}

        virtual std::optional<Eigen::VectorXd> evaluate_at(double t) const = 0;

        virtual double curr_time() final {return idx; };
        virtual void add_time() final {idx += dt; }

        std::optional<Eigen::Vector2d> linesearch(Eigen::Vector2d &center, Eigen::Matrix2d &cov, double t_init) const
        {

            // find the intersection point using mahalanobis
            Eigen::Vector2d t{t_init,0};
            while(t[0] <= 1)
            {
                auto eval = this->evaluate_at(t[0]);
                if (!eval.has_value()) return std::nullopt;
                if (mahalanobis(cov,center,eval.value().head<2>()) <= 9.0) break; 
                t[0] += dt;
            }
            
            t[1] = t[0] + dt;

            while(t[1] <= 1)
            {
                auto eval = this->evaluate_at(t[1]);
                if (!eval.has_value()) return std::nullopt;
                if (mahalanobis(cov,center,eval.value().head<2>()) > 9.0) break; 
                t[1] += dt;
            }
        
            return t;
        }
    private:
        double idx = 0;
    };

    class NominalTrajectory : public Trajectory {
    public:
        explicit NominalTrajectory(spline_testcase sp) : spl(std::move(sp)) {}


        std::optional<Eigen::VectorXd> evaluate_at(double t) const override {
            if (t > 1.0) return std::nullopt;

            Eigen::Vector2d pos;

            if (!spl.eval_at(t, pos)) return std::nullopt;

            Eigen::Vector2d xdot;
            spl.eval_at(t, xdot, 1);
            
            Eigen::Vector2d xdotdot;
            spl.eval_at(t, xdotdot, 2);

            double yaw = std::atan2(xdot[1], xdot[0]);
            double yawdot = (pow(cos(yaw), 2) / xdot[0]) * (xdotdot[1] - xdotdot[0] * tan(yaw));
            
            Eigen::VectorXd state(6);
            state<< pos, xdot, yaw, yawdot;

            return state;
        }
    private:
        spline_testcase spl;
    };

    class ReplannedTrajectory : public Trajectory {
    public:
        ReplannedTrajectory(tk::spline x, tk::spline y, double start_t, double end_t)
            : x_spline(std::move(x)), y_spline(std::move(y)), t0(t0), t1(t1) {}
        
        std::optional<Eigen::VectorXd> evaluate_at(double t) const override {
            if (t < t0 || t > t1) return std::nullopt;

            Eigen::VectorXd state(6);
            double xdot = x_spline.deriv(1, t);
            double ydot = y_spline.deriv(1, t);
            double yaw = std::atan2(ydot, xdot);
            double xdotdot = x_spline.deriv(2, t);
            double ydotdot = y_spline.deriv(2, t);
            double yawdot = (pow(cos(yaw), 2) / xdot) * (ydotdot - xdotdot * tan(yaw));

            state << x_spline(t), y_spline(t), 
                    yaw, xdot, ydot, yawdot;
            return state;
        }


    private:
        tk::spline x_spline, y_spline;
        double t0, t1;
    };

};