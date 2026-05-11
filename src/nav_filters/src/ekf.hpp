#ifndef __EKF_HPP__
#define __EKF_HPP__
#include "Eigen/Dense"
#include <iostream>
#include <fstream>

void rotational_mekf(const Eigen::Quaterniond &q_est_triad,
                     const Eigen::Vector3d &w_gyro,
                     const Eigen::VectorXd &prior_state,
                     const Eigen::Matrix<double,6,6> &prior_P,
                     double dt, double sigma_gyro,
                     Eigen::Matrix<double,6,6> &P,
                     Eigen::VectorXd &mekf_out)
{
    Eigen::Quaterniond prior_q(prior_state[0], prior_state[1], prior_state[2], prior_state[3]);
    Eigen::Vector3d prior_b(prior_state[4], prior_state[5], prior_state[6]);

    auto w_corrected = w_gyro - prior_b;
    Eigen::Quaterniond omega_quat(0, w_corrected[0], w_corrected[1], w_corrected[2]);
    Eigen::Quaterniond q_dot;
    q_dot.coeffs() = 0.5 * (prior_q * omega_quat).coeffs();
    Eigen::Quaterniond q_pred;
    q_pred.coeffs() = prior_q.coeffs() + q_dot.coeffs() * dt;
    q_pred.normalize();

    Eigen::Matrix3d w_x;
    w_x << 0, -w_corrected[2], w_corrected[1],
        w_corrected[2], 0, -w_corrected[0],
        -w_corrected[1], w_corrected[0], 0;

    Eigen::Matrix<double, 6, 6> F;
    F.topLeftCorner<3, 3>() = -w_x;
    F.topRightCorner<3, 3>() = -Eigen::Matrix3d::Identity();
    F.bottomLeftCorner<3, 3>() = Eigen::Matrix3d::Zero();
    F.bottomRightCorner<3, 3>() = Eigen::Matrix3d::Zero();

    const double sigma_arw = 1.10e-4;
    Eigen::Matrix3d Q_theta = Eigen::Matrix3d::Identity() * sigma_arw * sigma_arw;
    const double sigma_brw = 7.13e-5;
    Eigen::Matrix3d Qb = Eigen::Matrix3d::Identity() * sigma_brw * sigma_brw;
    Eigen::Matrix<double, 6, 6> Q;
    Q.setZero();
    Q.topLeftCorner<3, 3>() = Q_theta;
    Q.bottomRightCorner<3, 3>() = Qb;

    P = prior_P + (F * prior_P + prior_P * F.transpose() + Q) * dt;

    auto q_err = q_pred.conjugate() * q_est_triad;
    Eigen::Vector3d dtheta = 2.0 * q_err.vec();

    Eigen::Matrix<double, 3, 6> H;
    H << Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero();
    Eigen::Matrix3d R = 8.7e-4 * Eigen::Matrix3d::Identity();
    Eigen::Matrix3d S = H * P * H.transpose() + R;
    Eigen::Matrix<double, 6, 3> K = P * H.transpose() * S.inverse();
    Eigen::Matrix<double, 6, 1> dx = K * dtheta;

    const static Eigen::IOFormat CSVFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ",", "\n");

    std::ofstream innovations("rot_innovations.csv", std::ios::app);
    if (innovations.is_open()) {
        innovations << S.format(CSVFormat);
        innovations << "\n";
        innovations.close();
    }

    std::ofstream residuals("rot_residuals.csv", std::ios::app);
    if (residuals.is_open()) {
        residuals << dtheta.transpose().format(CSVFormat);
        residuals << "\n";
        residuals.close();
    }


    Eigen::Vector3d dtheta_corr = dx.head<3>();
    double angle = dtheta_corr.norm();
    Eigen::Quaterniond dq;
    if (angle < 1e-8) {
        dq = Eigen::Quaterniond(1, 0.5 * dtheta_corr[0], 0.5 * dtheta_corr[1], 0.5 * dtheta_corr[2]);
    } else {
        dq = Eigen::Quaterniond(Eigen::AngleAxisd(angle, dtheta_corr / angle));
    }

    Eigen::Quaterniond q = q_pred * dq;
    q.normalize();
    Eigen::Vector3d b = prior_b + dx.tail<3>();

    Eigen::Matrix<double, 6, 6> I6 = Eigen::Matrix<double, 6, 6>::Identity();
    Eigen::Matrix<double, 6, 6> IKH = I6 - K * H;
    P = IKH * P * IKH.transpose() + K * R * K.transpose();

    mekf_out.resize(7);
    mekf_out << q.w(), q.x(), q.y(), q.z(), b[0], b[1], b[2];
}

// Translational EKF — single-IMU version.
// State: [x, y, z, vx, vy, vz, yaw, dyaw, pitch, dpitch]   (10-D)
// Measurement z (5-D): [ax, ay, az, range, range_rate]
void translational_ekf(const Eigen::VectorXd &IMU_const,
                       const float_t range_const, const float_t range_rate_const,
                       const Eigen::Vector3d &orientation_estimate,
                       const Eigen::VectorXd &u,
                       const Eigen::VectorXd &prior_location,
                       const Eigen::MatrixXd &prior_P,
                       const Eigen::MatrixXd &rot_P,
                       double dt,
                       Eigen::Matrix<double, 15, 15> &P,
                       Eigen::VectorXd &state_hat)
{

    double rw = 0.17 / 2;
    double B  = 0.2;

    double wr  = u(0);
    double wl  = u(1);
    double dwr = u(2);
    double dwl = u(3);

    // tune this shit
    double mu_r = 1.0;
    double mu_l = 1.0;

    double v  = rw / 2 * (mu_r * wr + mu_l * wl);
    double dv = (rw / 2) * (mu_r * dwr + mu_l * dwl);

    double roll  = orientation_estimate[0];
    double pitch = orientation_estimate[1];
    double yaw   = orientation_estimate[2];

    Eigen::Matrix3d R_I_B;
    R_I_B << cos(pitch)*cos(yaw),  cos(yaw)*sin(roll)*sin(pitch) - cos(roll)*sin(yaw),  cos(roll)*cos(yaw)*sin(pitch) + sin(roll)*sin(yaw),
             cos(pitch)*sin(yaw),  cos(roll)*cos(yaw) + sin(roll)*sin(pitch)*sin(yaw),  cos(roll)*sin(pitch)*sin(yaw) - cos(yaw)*sin(roll),
            -sin(pitch),           cos(pitch)*sin(roll),                                cos(pitch)*cos(roll);

    Eigen::Vector3d body_frame_angvel(0, 0, rw / B * (mu_r * wr - mu_l * wl));
    Eigen::Vector3d inertial_angvel = R_I_B * body_frame_angvel;

    double droll  = inertial_angvel[0];
    double dpitch = inertial_angvel[1];
    double dyaw   = inertial_angvel[2];
    
    Eigen::VectorXd prior_estimate = Eigen::VectorXd::Zero(15);
    for (uint32_t i = 0; i < 6; i++) prior_estimate[i] = prior_location[i];

    prior_estimate[6] = pitch;
    prior_estimate[7] = dpitch;
    prior_estimate[8] = yaw;
    prior_estimate[9] = dyaw;

    prior_estimate[10] = prior_location[6];
    prior_estimate[11] = prior_location[7];
    prior_estimate[12] = prior_location[8];  

    prior_estimate[13] = prior_location[9];
    prior_estimate[14] = prior_location[10];
    
    Eigen::VectorXd IMU(3);
    IMU[0] = IMU_const[0] - prior_location[6];
    IMU[1] = IMU_const[1] - prior_location[7];
    IMU[2] = IMU_const[2] - prior_location[8];

    double range      = range_const      - prior_location[9];
    double range_rate = range_rate_const - prior_location[10];


    // IMU[0] = IMU_const[0];
    // IMU[1] = IMU_const[1];
    // IMU[2] = IMU_const[2];

    // double range      = range_const;
    // double range_rate = range_rate_const;
        
    double v_trans  = mu_r * wr + mu_l * wl;
    double dv_trans = mu_r * dwr + mu_l * dwl;
    double v_rot    = mu_r * wr - mu_l * wl;
    double dv_rot   = mu_r * dwr - mu_l * dwl;

    // partials wrt yaw
    double daxdyaw      = ( rw/2) * (sin(yaw) * sin(pitch) * dpitch * v_trans - 
                        cos(yaw) * cos(pitch) * dyaw * v_trans - 
                        cos(pitch) * sin(yaw) * dv_trans);
    
    double daydyaw      = ( rw/2) * (cos(pitch) * cos(yaw) * dv_trans - 
                        cos(yaw) * sin(pitch) * dpitch * v_trans - 
                        sin(yaw) * cos(pitch) * dyaw * v_trans);

    double dddpitchdyaw = (-rw/B) * ((cos(roll) * sin(yaw) * droll + 
                        cos(yaw) * sin(roll) * dyaw - 
                        cos(yaw) * sin(roll) * sin(pitch) * droll) * v_rot - 
                        (cos(roll) * sin(yaw) * sin(pitch) * dyaw + 
                        cos(roll) * cos(yaw) * cos(pitch) * dpitch) * dv_rot);
    
    
    // partial wrt dyaw

    double daxddyaw     = (-rw/2) * cos(pitch) * sin(yaw) * v_trans;
    
    double dayddyaw     = ( rw/2) * cos(yaw) * cos(pitch) * v_trans;

    double dddpitchddyaw= (-rw/B) * (sin(roll) * sin(yaw) + 
                        cos(roll) * cos(yaw) * sin(pitch)) * v_rot;

    double ddyawddyaw   = 1;

    // partials wrt pitch

    double daxdpitch    = ( rw/2) * (sin(yaw) * sin(pitch) * dyaw * v_trans - 
                        cos(yaw) * sin(pitch) * dv_trans - 
                        cos(yaw) * cos(pitch) * dpitch * v_trans);

    double daydpitch    = (-rw/2) * (sin(yaw) * sin(pitch) * dv_trans +
                        cos(yaw) * sin(pitch) * dyaw * v_trans +
                        cos(pitch) * sin(yaw) * dpitch * v_trans);
                
    double dazdpitch    = ( rw/2) * (sin(pitch) * dpitch * v_trans + 
                        cos(pitch) * dv_trans);

    double dddpitchdpitch=( rw/B) * v_rot * (cos(pitch) * sin(roll) * sin(yaw) * droll + 
                        cos(roll) * sin(yaw) * sin(pitch) * dpitch - 
                        cos(roll) * cos(yaw) * cos(pitch) * dyaw) - 
                        (rw/B) * dv_rot * (cos(roll) * cos(pitch) * sin(yaw));
    
    double dddyawdpitch = ( rw/B) * v_rot * (cos(roll) * cos(pitch) * dpitch - 
                        sin(roll) * sin(pitch) * droll) + 
                        ( rw/B) * dv_rot * (cos(roll) * sin(pitch));
    
    // partial wrt dpitch

    double daxddpitch   = (-rw/2) * v_trans * cos(yaw) * sin(pitch);

    double dayddpitch   = (-rw/2) * v_trans * sin(yaw) * sin(pitch);

    double dazddpitch   = (-rw/2) * v_trans * cos(pitch);
    
    double dddpitchddpitch = (-rw/B)*(cos(roll) * cos(pitch) * sin(yaw) * v_rot);

    double dddyawddpitch = ( rw/B) * cos(roll) * sin(pitch) * v_rot;

    double ddpitchddpitch = 1;
    
    // state_dot: accelerations (position, velocity, pdpydy)

    double ax      = (-rw/2) * v_trans * (cos(pitch) * sin(yaw) * dyaw + 
                cos(yaw) * sin(pitch) * dpitch) + 
                ( rw/2) * dv_trans * cos(yaw) * cos(pitch);
    double ay      = ( rw/2) * v_trans * (-sin(yaw) * sin(pitch) * dpitch + 
                cos(yaw) * cos(pitch) * dyaw) + 
                ( rw/2) * dv_trans * cos(pitch) * sin(yaw);
    
    double az      = (-rw/2) * (sin(pitch) * dv_trans + 
                cos(pitch) * dpitch * v_trans);
    
    // state_dot: angular accelerations

    double ddpitch = ( rw/B) * dv_rot * (cos(yaw) * sin(pitch) - 
                    cos(roll) * sin(yaw) * sin(pitch)) - 
                     ( rw/B) * v_rot * (-cos(roll) * cos(yaw) * droll + 
                    sin(roll) * sin(yaw) * dyaw + 
                    cos(roll) * cos(yaw) * sin(pitch) * dyaw + 
                    cos(roll) * cos(pitch) * sin(yaw) * dpitch - 
                    sin(roll) * sin(yaw) * sin(pitch) * droll);

    double ddyaw   = ( rw/B) * v_rot * (cos(pitch) * sin(roll) * droll - 
                    cos(roll) * sin(pitch) * dpitch) - 
                    ( rw/B) * cos(roll) * cos(pitch) * dv_rot;
    

    // //double daydyaw    = ( rw/2) * ((-dyaw * sin(yaw) - pitch * dpitch * cos(yaw)) * (wr + wl) + cos(yaw) * (dwr + dwl));
    // double dazdyaw    = 0;

    // double daxddyaw   = (-rw/2) * sin(yaw);
    // double dayddyaw   = ( rw/2) * cos(yaw);
    // double dazddyaw   = 0;

    // double daxdpitch  = (-rw/2) * ((dpitch * cos(yaw)) * (wr + wl));
    // double daydpitch  = ( rw/2) * ((-dpitch * sin(yaw)) * (wr + wl));
    // double dazdpitch  = ( rw/2) * (dwr + dwl);

    // double daxddpitch = (-rw/2) * ((pitch * cos(yaw)) * (wr + wl));
    // double dayddpitch = ( rw/2) * ((-pitch * sin(yaw)) * (wr + wl));
    // double dazddpitch = ( rw/2) * (wr + wl);

    // Process noise (diagonal). Zero on angle states — we trust the MEKF for those.
    Eigen::Matrix<double, 15, 15> W;
    W.setZero();
    W.diagonal() << pow(1e-3, 2), pow(1e-3, 2), pow(1e-3, 2),
                    pow(1e-3, 2), pow(1e-3, 2), pow(1e-3, 2),
                    pow(5e-2,2), pow(5e-2,2), pow(5e-2,2), pow(5e-2,2), //attitude
                    pow(1e-3,2), pow(1e-3,2), pow(1e-3,2), // bias accel
                    pow(1e-3,2), pow(1e-5,2);              // bias range range-rate
                    
    // W = W / pow(100, 2);
    // Measurement noise — 5x5 now. [ax, ay, az, range, range_rate]
    Eigen::Matrix<double, 5, 5> R;
    R.setZero();
    R.diagonal() << pow(0.01, 2), pow(0.03, 2), pow(0.01, 2),
                   pow(0.03, 2), pow(0.01, 2);

    // State derivative
    Eigen::VectorXd state_dot_hat(15);
    state_dot_hat.setZero();
    state_dot_hat << prior_location[3], prior_location[4], prior_location[5],
                     ax, ay, az,
                     dpitch, ddpitch, 
                     dyaw, ddyaw,
                     0,0,0, // bias accel   
                     0,0;   // bias range rangerate

    auto state_hat_pred = prior_estimate + state_dot_hat * dt;

    Eigen::Matrix<double, 15, 15> F;
    F.setZero();
    F << 
        0,0,0,1,0,0,0,0,0,0, /*bias terms -->*/ 0,0,0,0,0,
        0,0,0,0,1,0,0,0,0,0, /*bias terms -->*/ 0,0,0,0,0,
        0,0,0,0,0,1,0,0,0,0, /*bias terms -->*/ 0,0,0,0,0,
        0,0,0,0,0,0,daxdpitch,daxddpitch,daxdyaw,daxddyaw, /*bias terms -->*/ 0,0,0,0,0,
        0,0,0,0,0,0,daydpitch,dayddpitch,daydyaw,dayddyaw,/*bias terms -->*/ 0,0,0,0,0,
        0,0,0,0,0,0,dazdpitch,dazddpitch,0,0,/*bias terms -->*/ 0,0,0,0,0,
        0,0,0,0,0,0,0,1,0,0,/*bias terms -->*/ 0,0,0,0,0,
        0,0,0,0,0,0,dddpitchdpitch,dddpitchddpitch,dddpitchdyaw,dddpitchddyaw,/*bias terms -->*/ 0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,1,/*bias terms -->*/ 0,0,0,0,0,
        0,0,0,0,0,0,dddyawdpitch,dddyawddpitch,0,0,/*bias terms -->*/ 0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,1
        ;
        
    auto P_dot  = F * prior_P + prior_P * F.transpose() + W;
    auto P_pred = prior_P + P_dot * dt;

    double x_n     = state_hat_pred[0];
    double y_n     = state_hat_pred[1];
    double z_n     = state_hat_pred[2];
    double x_dot_n = state_hat_pred[3];
    double y_dot_n = state_hat_pred[4];
    double z_dot_n = state_hat_pred[5];
    double rho_n   = state_hat_pred.head<3>().norm();

    double alpha = prior_estimate.head<3>().dot(prior_estimate.segment<3>(3));

    // H is now 5x10 — three accel rows, one range row, one range-rate row.
    Eigen::Matrix<double, 5, 15> H;
    H.setZero();

    // Row 0: ax — depends on pitch, dpitch, yaw, dyaw, 
    H(0, 6) = daxdpitch;
    H(0, 7) = daxddpitch;
    H(0, 8) = daxdyaw;
    H(0, 9) = daxddyaw;
    // H(0,10) = 1;

    // Row 1: ay
    H(1, 6) = daydpitch;
    H(1, 7) = dayddpitch;
    H(1, 8) = daydyaw;
    H(1, 9) = dayddyaw;
    // H(1,11) = 1;
    
    // Row 2: az
    H(2, 6) = dazdpitch;
    H(2, 7) = dazddpitch;
    H(2, 8) = 0;
    H(2, 9) = 0;
    // H(2,12) = 1;

    // Row 3: range — depends on position
    H(3, 0) = x_n / rho_n;
    H(3, 1) = y_n / rho_n;
    H(3, 2) = z_n / rho_n;
    // H(3,13) = 1;

    // Row 4: range_rate — depends on position and velocity
    double rho_n2 = rho_n * rho_n;
    H(4, 0) = (x_dot_n * rho_n - alpha * x_n / rho_n) / rho_n2;
    H(4, 1) = (y_dot_n * rho_n - alpha * y_n / rho_n) / rho_n2;
    H(4, 2) = (z_dot_n * rho_n - alpha * z_n / rho_n) / rho_n2;
    H(4, 3) = x_n / rho_n;
    H(4, 4) = y_n / rho_n;
    H(4, 5) = z_n / rho_n;
    // H(4,14) = 1;

    // Kalman gain — 5x5 inversion now
    Eigen::Matrix<double, 5, 5> S = H * P_pred * H.transpose() + R;
    Eigen::Matrix<double, 15, 5> K = P_pred * H.transpose() * S.inverse();

    auto rho_pred   = state_hat_pred.head<3>().norm();
    auto alpha_pred = state_hat_pred.head<3>().dot(state_hat_pred.segment<3>(3));

    // Measurement vector and predicted measurement (5-D each)
    Eigen::Matrix<double, 5, 1> z;
    z << IMU(0), IMU(1), IMU(2), range, range_rate;

    // float_t ddx_pred = (-rw/2) * (dyaw * sin(yaw) + pitch * dpitch * cos(yaw) * (wr + wl) - cos(yaw) * (dwr + dwl));
    // float_t ddy_pred = ( rw/2) * (dyaw * cos(yaw) - pitch * dpitch * sin(yaw) * (wr + wl) + sin(yaw) * (dwr + dwl));
    // float_t ddz_pred = ( rw/2) * (dpitch * (wr + wl) + pitch * (dwr + dwl));

    // Eigen::Matrix<double, 5, 1> z_pred;
    Eigen::Matrix<double, 5, 1> z_pred;
    z_pred << ax ,
            ay ,
            az,
            rho_pred              ,
            alpha_pred / rho_pred   ;

    state_hat = state_hat_pred + K * (z - z_pred);
    auto res = z - z_pred;

    auto IKH = (Eigen::Matrix<double, 15, 15>::Identity() - K * H);
    P = IKH * P_pred * IKH.transpose() + K * R * K.transpose();
}

#endif