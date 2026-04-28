#ifndef __EKF_HPP__
#define __EKF_HPP__
#include "Eigen/Dense"

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
    auto b_pred = prior_b;

    Eigen::Matrix3d w_x;
    w_x << 0, -w_corrected[2], w_corrected[1],
        w_corrected[2], 0, -w_corrected[0],
        -w_corrected[1], w_corrected[0], 0;

    Eigen::Matrix<double, 6, 6> F;
    F.topLeftCorner<3, 3>() = -w_x;
    F.topRightCorner<3, 3>() = -Eigen::Matrix3d::Identity();
    F.bottomLeftCorner<3, 3>() = Eigen::Matrix3d::Zero();
    F.bottomRightCorner<3, 3>() = Eigen::Matrix3d::Zero();

    Eigen::Matrix3d Q_theta = Eigen::Matrix3d::Identity() * (4e-4) * (4e-4) * dt;
    Eigen::Matrix3d Qb = Eigen::Matrix3d::Identity() * (6.5e-5) * (6.5e-5) * dt;
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

    Eigen::Vector3d dtheta_corr = dx.head<3>();
    double angle = dtheta_corr.norm();
    Eigen::Quaterniond dq;
    if (angle < 1e-8)
    {
        dq = Eigen::Quaterniond(1, 0.5 * dtheta_corr[0], 0.5 * dtheta_corr[1], 0.5 * dtheta_corr[2]);
    }
    else
    {
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

void translational_ekf(const Eigen::VectorXd &IMU, const Eigen::Vector2d& UWB,
                       const Eigen::Vector3d &orientation_estimate, const Eigen::VectorXd& u, const Eigen::VectorXd& prior_estimate,
                       const Eigen::MatrixXd &prior_P, const Eigen::MatrixXd &rot_P, double dt,
                       Eigen::Matrix<double,6,6> &P, Eigen::VectorXd &state_hat)
{
    double rw = 0.17 / 2;

    double wr = u(0);
    double wl = u(1);
    double dwr = u(2);
    double dwl = u(3);

    // diff drive kinematics
    double v = rw / 2 * (wr + wl);
    double dv = (rw / 2) * (dwr + dwl) / dt;

    double pitch = orientation_estimate[1];
    double yaw = orientation_estimate[2];

    Eigen::Matrix<double, 6, 6> W;
    W.setZero();
    W.diagonal() << 0.001, 0.001, 0.001, 0.0001, 0.0001, 0.0001;

    Eigen::Matrix<double, 8, 8> R;
    R.setZero();
    R.diagonal() << pow(0.03, 2), pow(0.03, 2), pow(0.03, 2), pow(0.003, 2), pow(0.003, 2), pow(0.003, 2), pow(0.01, 2), pow(0.001, 2);

    Eigen::VectorXd state_dot_hat(6);
    state_dot_hat << cos(yaw) * v, sin(yaw) * v, sin(pitch) * v,
        cos(yaw) * dv, sin(yaw) * dv, sin(pitch) * dv;

    auto state_hat_pred = prior_estimate + state_dot_hat * dt;

    // cov propagation
    auto P_dot = W;
    auto P_pred = prior_P + P_dot * dt;

    double x_n = state_hat_pred[0];
    double y_n = state_hat_pred[1];
    double z_n = state_hat_pred[2];
    double x_dot_n = state_hat_pred[3];
    double y_dot_n = state_hat_pred[4];
    double z_dot_n = state_hat_pred[5];
    double rho_n = state_hat_pred.head<3>().norm();

    double alpha = prior_estimate.head<3>().dot(prior_estimate.tail<3>());

    Eigen::Matrix<double, 8, 6> H;
    H.setZero();
    H.topLeftCorner<6, 6>() = Eigen::Matrix<double, 6, 6>::Identity();
    H(6, 0) = x_n / rho_n;
    H(6, 1) = y_n / rho_n;
    H(6, 2) = z_n / rho_n;

    double rho_n2 = rho_n * rho_n;
    H(7, 0) = (x_dot_n * rho_n - alpha * x_n / rho_n) / rho_n2;
    H(7, 1) = (y_dot_n * rho_n - alpha * y_n / rho_n) / rho_n2;
    H(7, 2) = (z_dot_n * rho_n - alpha * z_n / rho_n) / rho_n2;
    H(7, 3) = x_n / rho_n;
    H(7, 4) = y_n / rho_n;
    H(7, 5) = z_n / rho_n;

    auto K = P_pred * H.transpose() * (H * P_pred * H.transpose() + R).completeOrthogonalDecomposition().pseudoInverse();
    auto rho_pred = state_hat_pred.head<3>().norm();
    auto alpha_pred = state_hat_pred.head<3>().dot(state_hat_pred.tail<3>());

    
    Eigen::VectorXd z(IMU.size() + UWB.size());
    z << IMU, UWB;
    Eigen::VectorXd z_pred(state_hat_pred.size() + 2);
    z_pred << state_hat_pred, rho_pred, alpha_pred / rho_pred;

    state_hat = state_hat_pred + K * (z - z_pred);
    P = (Eigen::Matrix<double, 6, 6>::Identity() - K * H) * P_pred;
}
#endif