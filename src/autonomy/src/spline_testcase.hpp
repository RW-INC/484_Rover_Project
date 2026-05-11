#ifndef __SPLINE_CASE__
#define __SPLINE_CASE__

#include "planning.hpp"
#include "Eigen/Dense"

#include <string>
#include <sstream>
#include <vector>

class spline_testcase
{
public:
    struct funct
    {
        Eigen::Vector4d x_coeffs;
        Eigen::Vector4d y_coeffs;
        double start_time = 0.0;
        double end_time = 0.0;

        bool case_eval_at(double t, Eigen::Vector2d &result, int order = 0) const
        {
            if (t < this->start_time || t > this->end_time)
                return false;
            double dt = t - this->start_time;
            double dt2 = dt * dt;
            double dt3 = dt2 * dt;

            Eigen::Vector4d pows;
            switch (order)
            {
            case 0:
                pows << dt3, dt2, dt, 1.0;
                break;
            case 1:
                pows << 3.0 * dt2, 2.0 * dt, 1.0, 0.0;
                break;
            case 2:
                pows << 6.0 * dt, 2.0, 0.0, 0.0;
                break;
            case 3:
                pows << 6.0, 0.0, 0.0, 0.0;
                break;
            default:
                return false;
            }

            result[0] = this->x_coeffs.dot(pows);
            result[1] = this->y_coeffs.dot(pows);
            return true;
        }
    };

private:
    std::vector<funct> vec;

public:
    const std::vector<funct> &get_vec() const { return vec; }

    bool eval_at(double t, Eigen::Vector2d &result, int order = 0) const
    {
        // O(n) bin search; n is small (one segment per spline piece) so it's fine.
        for (auto &item : this->vec)
            if (t >= item.start_time && t <= item.end_time)
                return item.case_eval_at(t, result, order);
        return false;
    }

    void parse_testcase(std::string data)
    {
        this->vec.clear();
        std::stringstream ss(data);

        uint32_t n_lines = 0;
        ss >> n_lines;

        double start_time = 0.0;
        while (n_lines-- > 0)
        {
            funct sp;
            for (uint32_t i = 0; i < 4; i++)
                ss >> sp.x_coeffs[i];
            for (uint32_t i = 0; i < 4; i++)
                ss >> sp.y_coeffs[i];

            sp.start_time = start_time;
            ss >> sp.end_time;
            start_time = sp.end_time;

            this->vec.push_back(sp);
        }
    }
};

#endif