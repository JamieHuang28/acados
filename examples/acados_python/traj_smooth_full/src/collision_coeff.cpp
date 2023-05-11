#include "collision_coeff.h"

namespace acados_main
{

Eigen::VectorXd GetSideCollisionCoeff(double xn, double yn, double x0, double y0, double phi, double slack_max, double d_safe, double L) {
    double c1 = xn;
    double c2 = yn;
    double c3 = yn * L * std::cos(phi) - xn * L * std::sin(phi);
    double d = -slack_max - d_safe + x0 * xn + y0 * yn - L * (std::cos(phi) + phi * std::sin(phi)) * xn - \
        L * (std::sin(phi) - phi * std::cos(phi)) * yn;
    Eigen::VectorXd coeff(10);
    coeff << c1, c2, c3, 0.0, 0.0,   0.0, 0.0, -1.0, 0.0,   d;
    return coeff;
}

Eigen::VectorXd GetFrontCollisionCoeff(double xn, double yn, double x0, double y0, double phi, double slack_max, double d_safe, double L, double W) {
    double c1 = xn;
    double c2 = yn;
    double c3 = yn * L * std::cos(phi) - xn * L * std::sin(phi) - W / 2.0;
    double d = -slack_max - d_safe + x0 * xn + y0 * yn - L * (std::cos(phi) + phi * std::sin(phi)) * xn - \
        L * (std::sin(phi) - phi * std::cos(phi)) * yn - phi * W / 2.0;
    Eigen::VectorXd coeff(10);
    coeff << c1, c2, c3, 0.0, 0.0,   0.0, 0.0, 0.0, -1.0,   d;
    return coeff;
}

Eigen::VectorXd GetBackCollisionCoeff(double xn, double yn, double x0, double y0, double  phi, double slack_max, double d_safe, double L, double W) {
    double c1 = xn;
    double c2 = yn;
    double c3 = yn * L * std::cos(phi) - xn * L * std::sin(phi) + W / 2.0;
    double d = -slack_max - d_safe + x0 * xn + y0 * yn - L * (std::cos(phi) + phi * std::sin(phi)) * xn - \
        L * (std::sin(phi) - phi * std::cos(phi)) * yn + phi * W / 2.0;
    Eigen::VectorXd coeff(10);
    coeff << c1, c2, c3, 0.0, 0.0,   0.0, 0.0, 0.0, -1.0,   d;
    return coeff;
}

Eigen::MatrixXd GetCollisionCoeff(const AcadosParams params_,
const Eigen::VectorXd &x,
                                        const Eigen::VectorXd &y,
                                        const Eigen::VectorXd &phi,
                                        const Eigen::VectorXd &v,
                                        const std::vector<double> &left_bounds,
                                        const std::vector<double> &right_bounds,
                                        const std::vector<double> &front_bounds,
                                        const std::vector<double> &back_bounds) {
  double vehicleWidth = params_.vehicleWidth;
  double wheelBase = params_.vehicleLength;
  double Lf = params_.Lf;
  double Lr = params_.Lr;
  double slack_max = params_.slack_max;
  double d_safe_side = params_.d_safe_side;
  double d_safe_front = params_.d_safe_front;

  Eigen::MatrixXd collision_coeff(8 * PATH_SMOOTHER_N, 10);
  for (int i = 0; i < PATH_SMOOTHER_N; ++i) {
    {
    // for left side
    double xn = std::cos(phi[i] + M_PI / 2.0);
    double yn = std::sin(phi[i] + M_PI / 2.0);
    double xv =  0;
    double yv = left_bounds[i];
    double x0 = x[i] + xv * std::cos(phi[i]) - yv * std::sin(phi[i]);
    double y0 = y[i] + xv * std::sin(phi[i]) + yv * std::cos(phi[i]);
    collision_coeff.row(8 * i + 0) = GetSideCollisionCoeff(xn, yn, x0, y0, phi[i], slack_max, d_safe_side, Lf);
    collision_coeff.row(8 * i + 1) = GetSideCollisionCoeff(xn, yn, x0, y0, phi[i], slack_max, d_safe_side, -Lr);
    }
    {
    // for right side
    double xn = std::cos(phi[i] - M_PI / 2.0);
    double yn = std::sin(phi[i] - M_PI / 2.0);
    double xv =  0;
    double yv = -right_bounds[i];
    double x0 = x[i] + xv * std::cos(phi[i]) - yv * std::sin(phi[i]);
    double y0 = y[i] + xv * std::sin(phi[i]) + yv * std::cos(phi[i]);
    collision_coeff.row(8 * i + 2) = GetSideCollisionCoeff(xn, yn, x0, y0, phi[i], slack_max, d_safe_side, Lf);
    collision_coeff.row(8 * i + 3) = GetSideCollisionCoeff(xn, yn, x0, y0, phi[i], slack_max, d_safe_side, -Lr);
    }
    {
    // for front side
    double xn = std::cos(phi[i]);
    double yn = std::sin(phi[i]);
    double xv = front_bounds[i];
    double yv = 0;
    double x0 = x[i] + xv * std::cos(phi[i]) - yv * std::sin(phi[i]);
    double y0 = y[i] + xv * std::sin(phi[i]) + yv * std::cos(phi[i]);
    collision_coeff.row(8 * i + 4) = GetFrontCollisionCoeff(xn, yn, x0, y0, phi[i], slack_max, d_safe_front, Lf, vehicleWidth);
    collision_coeff.row(8 * i + 5) = GetBackCollisionCoeff(xn, yn, x0, y0, phi[i], slack_max, d_safe_front, Lf, vehicleWidth);
    }
    {
    // for rear side
    double xn = std::cos(phi[i] - M_PI);
    double yn = std::sin(phi[i] - M_PI);
    double xv = -back_bounds[i];
    double yv = 0;
    double x0 = x[i] + xv * std::cos(phi[i]) - yv * std::sin(phi[i]);
    double y0 = y[i] + xv * std::sin(phi[i]) + yv * std::cos(phi[i]);
    collision_coeff.row(8 * i + 6) = GetFrontCollisionCoeff(xn, yn, x0, y0, phi[i], slack_max, d_safe_front, -Lr, vehicleWidth);
    collision_coeff.row(8 * i + 7) = GetBackCollisionCoeff(xn, yn, x0, y0, phi[i], slack_max, d_safe_front, -Lr, vehicleWidth);
    }
  }

  return collision_coeff;
}

} // namespace acados_main