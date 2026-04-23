#ifndef __TESTCASE_HPP__
#define __TESTCASE_HPP__


#include "../spline/src/spline.h"
#include "../Planning/grid2d.hpp"
#include "../eigen/Eigen/Dense"

// #define M_PI 3.14159265358979323846
#define MAX_VEL 0.02f  // cm/s
#define MIN_VEL 0.014f //cm/s
#define EPS 1e-9f 
// #define roundto(x,n) std::round(x * std::pow(10,n))/std::pow(10,n) // rounds x to n decimal places

struct imu {
    float_t SAMPLE_HZ;                      // sampling frequency of the IMU
    float_t ACCEL_VRW_MPS_SQRT_HZ;          // velocity random walk of the accelerometer in m/s/sqrt(Hz)
    float_t GYRO_VRW_DEG_SQRT_HZ;           // velocity random walk of the gyroscope in deg/s/sqrt(Hz)
    float_t ACCEL_BIAS_INSTABILITY_MG;      // bias drift of the accelerometer in m/s/sqrt(Hz)
    float_t GYRO_BIAS_INSTABILITY_DEG_HR;   // bias drift of the gyroscope in deg/s/sqrt(Hz)
};

struct sunsensor {
    float_t SUN_UPDATE_PERIOD;              // how often to update the sun sensor reading in seconds
    float_t SUN_VECTOR_MEAS_STD;            // standard deviation of the sun vector measurement noise in degrees
    bool    USE_SUN_SENSOR;                 // whether to use a sun sensor for orientation estimation
};

struct rangesensor {
    float_t RANGE_UPDATE_PERIOD;            // how often to update the range sensor reading in seconds
    float_t RANGE_NOISE_STD;                // standard deviation of the range measurement noise in meters
    bool    USE_RANGE_SENSOR;               // whether to use a range sensor for altitude estimation
};

struct surface {
    float_t SURFACE_UPDATE_PERIOD;          // how often to update the surface measurement in seconds
    float_t SURFACE_NOISE_STD;              // standard deviation of the surface measurement noise in meters
    float_t SURFACE_Z_MEAS_STD = 1e-4;      // standard deviation of the surface measurement noise in meters 
    float_t SURFACE_VZ_MEAS_STD = 1e-4;     // standard deviation of the surface measurement noise in meters per second
    bool    USE_SURFACE_SENSOR;             // whether to use a surface sensor for altitude estimation
};

struct gravity {
    float_t GRAVITY_UPDATE_PERIOD;          // how often to update the gravity vector in seconds
    float_t GRAVITY_VECTOR_MEAS_STD;        // standard deviation of the gravity vector measurement noise in degrees
    bool USE_GRAVITY_UPDATE;                // whether to update the gravity vector during the simulation
};

struct init {
    float_t INIT_POS_STD = 0.0;                // standard deviation of the initial position error in meters
    float_t INIT_VEL_STD = 0.0;                // standard deviation of the initial velocity error in m/s
    float_t INIT_ATT_STD = 5.0 * M_PI / 180.0; // standard deviation of the initial orientation error in degrees
    float_t INIT_BA_STD = 5.0e-3;              // standard deviation of the initial accelerometer bias error in m/s^2
    float_t INIT_BG_STD = 0.1 * M_PI / 180.0;  // standard deviation of the initial gyroscope bias error in deg/s
};

struct ekf {
    float_t EKF_ACCEL_PROCESS_STD = 5.0e-3;              // how often to update the EKF in seconds
    float_t EKF_GYRO_PROCESS_STD = 0.05 * M_PI / 180.0;  // standard deviation of the EKF process noise for the gyroscope in deg/s^2
    float_t EKF_ACCEL_BIAS_RW_STD = 1.0e-5;              // standard deviation of the EKF process noise for the accelerometer bias in m/s^3
    float_t EKF_GYRO_BIAS_RW_STD = 5e-5 * M_PI / 180.0;  // standard deviation of the EKF process noise for the gyroscope bias in deg/s^2
    bool USE_EKF;                                        // whether to use an EKF for state estimation
};

struct spline {
    uint32_t SPLINE_NUM_POINTS;                // number of points to use for the spline interpolation of the trajectory
    tk::spline SPLINE_INTERPOLATOR;            // the spline interpolator object from the tk::spline library, used for generating smooth trajectories  
    std::vector<double_t> X_SPLINE_POINTS;     // the x coordinates of the points used for the spline interpolation
    std::vector<double_t> Y_SPLINE_POINTS;     // the y coordinates of the points used for the spline interpolation
};

/**
 * Defines simconfig, noise additions to trajectory, IMU noise, and various other parameters for the simulation.
 * This is a bit of a catch-all for now, but it should be easy to expand as we go along.
 */
struct SimConfig
{
    uint32_t NUM_MC_RUNS;

    std::vector<float_t> t_vec;                  // vector of time steps for the trajectory, used for generating sensor readings and noise
    imu* imu;                                     // pointer to the IMU, which is used for simulating sensor readings
    planning_module::grid2d* grid;               // pointer to the grid, which is used for collision checking and path planning    

    planning_module::coordf lander_position;     // position of the lander in the grid, used for range sensor simulation
    Eigen::Matrix2Xf  rover_body_tags{           // the trajectory to be followed, represented as a 2xN matrix of (x, y) positions at each time step
        {0.0, 0.0, 0.0},
        {0.5, 0.0, 0.0}
    };         
};

#include <vector>
#include <iostream>
#include <random>
#include <algorithm>

std::unique_ptr<spline> generate_testcases(SimConfig &cfg, float_t x0 = 0.0, float_t y0 = 0.0)
{
    auto s = std::make_unique<spline>();
    s->SPLINE_NUM_POINTS = cfg.t_vec.size();
   
    auto t_knots = Eigen::VectorXd::LinSpaced(s->SPLINE_NUM_POINTS, cfg.t_vec.front(), cfg.t_vec.back());
    
    // use the min velocity and get the max radius travelled 
    // get a random float between min and max velocity
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float_t> dis(MIN_VEL, MAX_VEL + EPS); //include MAX_VEL, we'll truncate the decimal anyways to 2 decimal places. 

    s->X_SPLINE_POINTS.resize(s->SPLINE_NUM_POINTS);
    s->Y_SPLINE_POINTS.resize(s->SPLINE_NUM_POINTS);
    
    s->X_SPLINE_POINTS[0] = x0;
    s->Y_SPLINE_POINTS[0] = y0;
    
    for (uint32_t i = 1; i < s->SPLINE_NUM_POINTS; i++)
    {
        auto dt = t_knots(i) - t_knots(i-1);
        auto vel = dis(gen);
        auto r = vel * (dt);     
        
        // generate som random angle between 45 and 135 degrees. gets nicer curves than assuming the most limiting case of 
        // -90 and 90 deg
        auto angle = std::uniform_real_distribution<float_t>(-M_PI / 8, M_PI / 8)(gen);
        // i added more decimal places because of eps, but i doubt i have that much data...
        // lets just conservatively round to 3 decimal places and call it a day.
        s->X_SPLINE_POINTS[i] = s->X_SPLINE_POINTS[i-1] + r * std::cos(angle);
        s->Y_SPLINE_POINTS[i] = s->Y_SPLINE_POINTS[i-1] + r * std::sin(angle);
    }

    // and we can use x,y to generate the spline
    s->SPLINE_INTERPOLATOR.set_points(s->X_SPLINE_POINTS, s->Y_SPLINE_POINTS);
    return s;
}

// std::map<std::string, SimConfig> run_mc(SimConfig &cfg)
// {
//  ;   
// }





#endif