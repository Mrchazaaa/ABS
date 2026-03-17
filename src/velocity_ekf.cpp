#include "velocity_ekf.h"

#include <cmath>
#include <cstring>

#include "tinyekf_config.h"

#define _float_t double
extern "C" {
#include "tinyekf.h"
}

namespace {

constexpr double kMass = 1265.0;
constexpr double kWheelRadius = 0.3179;
constexpr double kGravity = 9.81;
constexpr double kC1 = 1.28;
constexpr double kC2 = 23.99;
constexpr double kC3 = 0.52;
constexpr double kDiffStep = 1e-5;

ekf_t g_ekf;
double g_q[EKF_N * EKF_N];
double g_r[EKF_M * EKF_M];
double g_input[EKF_M];

static double clamp_abs(double value)
{
  return std::fabs(value) < 1e-6 ? (value < 0.0 ? -1e-6 : 1e-6) : value;
}

static void init_covariance()
{
  std::memset(g_q, 0, sizeof(g_q));
  std::memset(g_r, 0, sizeof(g_r));
  g_q[0] = 0.1;
  g_q[4] = 0.1;
  g_q[8] = 0.1;
  g_r[0] = 0.00001;
  g_r[4] = 0.01;
  g_r[8] = 0.01;
}

static void state_transition(
    const double state[EKF_N],
    const double input[EKF_M],
    double out[EKF_N])
{
  const double x_velocity = clamp_abs(state[0]);
  const double y_velocity = state[1];
  const double surface_scale = state[2];

  const double driving_angle = input[0];
  const double front_wheel_vel = input[1];
  const double rear_wheel_vel = input[2];

  const double alphaf = driving_angle - std::atan(y_velocity / x_velocity);
  const double alphar = -std::atan(y_velocity / x_velocity);

  const double vf = std::sqrt(y_velocity * y_velocity + x_velocity * x_velocity);
  const double vr = vf;
  const double safe_vf = clamp_abs(vf);
  const double safe_vr = clamp_abs(vr);

  const double sxf = -((kWheelRadius * front_wheel_vel * std::cos(alphaf)) - vf) / safe_vf;
  const double sxr = ((kWheelRadius * rear_wheel_vel * std::cos(alphar)) - vr) / safe_vr;
  const double syf = (front_wheel_vel * kWheelRadius * std::sin(alphaf)) / safe_vf;
  const double syr = (rear_wheel_vel * kWheelRadius * std::sin(alphar)) / safe_vr;

  const double sresf = std::sqrt(sxf * sxf + syf * syf);
  const double sresr = std::sqrt(sxr * sxr + syr * syr);

  const double fzf = (kMass / 2.0) * kGravity;
  const double fzr = (kMass / 2.0) * kGravity;

  const double muresf = ((kC1 * (1 - std::exp(-kC2 * sresf))) - (kC3 * sresf)) * surface_scale;
  const double muresr = ((kC1 * (1 - std::exp(-kC2 * sresr))) - (kC3 * sresr)) * surface_scale;

  double fxf = 0.0;
  double fyf = 0.0;
  if (sresf > 0.001) {
    fxf = (muresf / sresf) * fzf * sxf;
    fyf = (muresf / sresf) * fzf * syf;
  }

  double fxr = 0.0;
  double fyr = 0.0;
  if (sresr > 0.001) {
    fxr = (muresr / sresr) * fzr * sxr;
    fyr = (muresr / sresr) * fzr * syr;
  }

  const double x_velocity_deriv =
      (((fxf * std::cos(driving_angle)) - (fyf * std::sin(driving_angle)) + fxr) / kMass);
  const double y_velocity_deriv =
      (((fyf * std::cos(driving_angle)) + (fxf * std::sin(driving_angle)) + fyr) / kMass);

  out[0] = state[0] + x_velocity_deriv;
  out[1] = state[1] + y_velocity_deriv;
  out[2] = state[2];
}

static void finite_difference_jacobian(
    const double state[EKF_N],
    const double input[EKF_M],
    double jacobian[EKF_N * EKF_N])
{
  double baseline[EKF_N];
  state_transition(state, input, baseline);

  for (int column = 0; column < EKF_N; ++column) {
    double perturbed_state[EKF_N] = {state[0], state[1], state[2]};
    perturbed_state[column] += kDiffStep;

    double perturbed[EKF_N];
    state_transition(perturbed_state, input, perturbed);

    for (int row = 0; row < EKF_N; ++row) {
      jacobian[row * EKF_N + column] =
          (perturbed[row] - baseline[row]) / kDiffStep;
    }
  }
}

}  // namespace

int abs_start_ekf(double initial_velocity)
{
  const double pdiag[EKF_N] = {1000000.0, 1000000.0, 1000000.0};
  ekf_initialize(&g_ekf, pdiag);
  init_covariance();

  g_ekf.x[0] = initial_velocity - 0.01;
  g_ekf.x[1] = 0.01;
  g_ekf.x[2] = 1.1;
  return 0;
}

double abs_step_ekf(
    double driving_angle,
    double front_wheel_vel,
    double rear_wheel_vel)
{
  g_input[0] = driving_angle;
  g_input[1] = front_wheel_vel;
  g_input[2] = rear_wheel_vel;

  double current_state[EKF_N] = {
      g_ekf.x[0],
      g_ekf.x[1],
      g_ekf.x[2],
  };
  double predicted_state[EKF_N];
  double jacobian[EKF_N * EKF_N];
  state_transition(current_state, g_input, predicted_state);
  finite_difference_jacobian(current_state, g_input, jacobian);

  ekf_predict(&g_ekf, predicted_state, jacobian, g_q);

  const double measurement[EKF_M] = {g_input[0], g_input[1], g_input[2]};
  const double measurement_prediction[EKF_M] = {
      g_input[0], g_input[1], g_input[2]};
  const double measurement_jacobian[EKF_M * EKF_N] = {0.0};
  ekf_update(&g_ekf, measurement, measurement_prediction, measurement_jacobian, g_r);

  return g_ekf.x[0];
}
