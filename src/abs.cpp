#include "ABS/abs.h"

#include <cmath>
#include <cstdio>
#include <cstring>

#include "velocity_ekf.h"

namespace {

constexpr int kPhaseOff = 0;

const AbsConfig kConfig = {
    10.0f,
    10.0f,
    13000.0f,
    0.04f,
    11000000.0f,
    8458000.0f,
    50000000.0f,
    -95.0f,
    0.0f,
    0.12f,
    13000000.0f,
    0.3179f,
};

static float clamp_nonzero(float value)
{
  if (std::fabs(value) >= 1e-6f) {
    return value;
  }
  return value < 0.0f ? -1e-6f : 1e-6f;
}

}  // namespace

struct AbsController {
  float timers[ABS_WHEEL_COUNT];
  int phase_states[ABS_WHEEL_COUNT];
  float wheel_spin_velocity[ABS_WHEEL_COUNT];
  float wheel_spin_acceleration[ABS_WHEEL_COUNT];
  float wheel_slip_acceleration[ABS_WHEEL_COUNT];
  float wheel_slip[ABS_WHEEL_COUNT];
  float max_wheel_slip[ABS_WHEEL_COUNT];
  float brake_command[ABS_WHEEL_COUNT];
  float last_timestamp;
  float delta_time;
  float vehicle_speed;
  int ekf_is_active;
};

static void reset_runtime_state(AbsController *controller)
{
  for (int i = 0; i < ABS_WHEEL_COUNT; ++i) {
    controller->timers[i] = -1.0f;
    controller->phase_states[i] = kPhaseOff;
    controller->wheel_spin_velocity[i] = 0.0f;
    controller->wheel_spin_acceleration[i] = 0.0f;
    controller->wheel_slip_acceleration[i] = 0.0f;
    controller->wheel_slip[i] = 0.0f;
    controller->max_wheel_slip[i] = kConfig.initial_max_wheel_slip;
    controller->brake_command[i] = 0.0f;
  }
  controller->last_timestamp = 0.0f;
  controller->delta_time = 0.0f;
  controller->vehicle_speed = 0.0f;
  controller->ekf_is_active = 0;
}

static void apply_phase(
    AbsController *controller,
    int wheel,
    float input_pressure)
{
  switch (controller->phase_states[wheel]) {
    case kPhaseOff:
      controller->phase_states[wheel] = 1;
      break;
    case 1:
      controller->brake_command[wheel] = input_pressure;
      if (kConfig.min_wheel_spin_acceleration >
          controller->wheel_spin_acceleration[wheel]) {
        controller->phase_states[wheel] = 2;
      }
      break;
    case 2:
      if (controller->wheel_slip[wheel] > controller->max_wheel_slip[wheel]) {
        controller->max_wheel_slip[wheel] = controller->wheel_slip[wheel];
        controller->phase_states[wheel] = 3;
      }
      break;
    case 3: {
      const float pressure_to_release = controller->delta_time * kConfig.release_rate;
      const float cmd_release_pressure = pressure_to_release / kConfig.max_brake_pressure;
      controller->brake_command[wheel] -= cmd_release_pressure;
      if (controller->wheel_spin_acceleration[wheel] >
          kConfig.max_wheel_spin_acceleration) {
        controller->phase_states[wheel] = 4;
      }
      break;
    }
    case 4:
      if (controller->timers[wheel] < 0.0f) {
        controller->timers[wheel] = kConfig.apply_delay;
      } else {
        controller->timers[wheel] -= controller->delta_time;
      }
      if (controller->timers[wheel] < 0.0f ||
          (kConfig.max_wheel_spin_acceleration * 10.0f) <
              controller->wheel_spin_acceleration[wheel]) {
        controller->timers[wheel] = -1.0f;
        controller->phase_states[wheel] = 5;
      }
      break;
    case 5: {
      const float pressure_to_apply =
          controller->delta_time * kConfig.primary_apply_rate;
      const float cmd_apply_pressure = pressure_to_apply / kConfig.max_brake_pressure;
      controller->brake_command[wheel] += cmd_apply_pressure;
      if (controller->wheel_spin_acceleration[wheel] < 0.0f) {
        controller->phase_states[wheel] = 6;
      }
      break;
    }
    case 6:
      if (controller->timers[wheel] < 0.0f) {
        controller->timers[wheel] = kConfig.apply_delay;
      } else {
        controller->timers[wheel] -= controller->delta_time;
      }
      if (controller->timers[wheel] < 0.0f ||
          kConfig.min_wheel_spin_acceleration >
              controller->wheel_spin_acceleration[wheel]) {
        controller->timers[wheel] = -1.0f;
        controller->phase_states[wheel] = 7;
      }
      break;
    case 7: {
      const float pressure_to_apply =
          controller->delta_time * kConfig.secondary_apply_rate;
      const float cmd_apply_pressure = pressure_to_apply / kConfig.max_brake_pressure;
      if (controller->brake_command[wheel] >= 1.0f) {
        controller->brake_command[wheel] = 1.0f;
      } else {
        controller->brake_command[wheel] += cmd_apply_pressure;
      }
      if (kConfig.min_wheel_spin_acceleration >
          controller->wheel_spin_acceleration[wheel]) {
        controller->phase_states[wheel] = 3;
      }
      break;
    }
    default:
      std::printf("ABS state error\n");
      break;
  }
}

static void copy_debug_state(
    const AbsController *controller,
    AbsStepOutput *output)
{
  for (int i = 0; i < ABS_WHEEL_COUNT; ++i) {
    output->brake_command[i] = controller->brake_command[i];
    output->debug.phase_states[i] = controller->phase_states[i];
    output->debug.wheel_spin_velocity[i] = controller->wheel_spin_velocity[i];
    output->debug.wheel_spin_acceleration[i] =
        controller->wheel_spin_acceleration[i];
    output->debug.wheel_slip_acceleration[i] =
        controller->wheel_slip_acceleration[i];
    output->debug.wheel_slip[i] = controller->wheel_slip[i];
    output->debug.max_wheel_slip[i] = controller->max_wheel_slip[i];
  }
  output->debug.delta_time = controller->delta_time;
  output->debug.vehicle_speed = controller->vehicle_speed;
}

const AbsConfig *abs_get_config(void)
{
  return &kConfig;
}

AbsController *abs_create(void)
{
  AbsController *controller = new AbsController();
  reset_runtime_state(controller);
  return controller;
}

void abs_destroy(AbsController *controller)
{
  delete controller;
}

void abs_reset(AbsController *controller)
{
  if (controller == nullptr) {
    return;
  }
  reset_runtime_state(controller);
}

void abs_step(
    AbsController *controller,
    const AbsStepInput *input,
    AbsStepOutput *output)
{
  if (controller == nullptr || input == nullptr || output == nullptr) {
    return;
  }

  if (controller->last_timestamp == 0.0f) {
    controller->ekf_is_active = 0;
    controller->last_timestamp = input->timestamp;
  }

  controller->delta_time = input->timestamp - controller->last_timestamp;
  if (controller->delta_time <= 0.0f) {
    controller->delta_time = 1e-6f;
  }

  for (int i = 0; i < ABS_WHEEL_COUNT; ++i) {
    controller->wheel_spin_acceleration[i] =
        (input->wheel_spin_velocity[i] - controller->wheel_spin_velocity[i]) /
        controller->delta_time;
    controller->wheel_spin_velocity[i] = input->wheel_spin_velocity[i];
  }

  if (controller->ekf_is_active == 0) {
    controller->vehicle_speed =
        ((input->wheel_spin_velocity[ABS_FL] * kConfig.wheel_radius_static) +
         (input->wheel_spin_velocity[ABS_FR] * kConfig.wheel_radius_static)) / 2.0f;
  } else {
    const double front_angular_speed =
        (input->wheel_spin_velocity[ABS_FL] +
         input->wheel_spin_velocity[ABS_FR]) / 2.0;
    const double rear_angular_speed =
        (input->wheel_spin_velocity[ABS_RL] +
         input->wheel_spin_velocity[ABS_RR]) / 2.0;
    controller->vehicle_speed = static_cast<float>(
        abs_step_ekf(0.0, front_angular_speed, rear_angular_speed));
  }

  const float safe_vehicle_speed = clamp_nonzero(controller->vehicle_speed);
  for (int i = 0; i < ABS_WHEEL_COUNT; ++i) {
    const float next_slip =
        (controller->vehicle_speed -
         controller->wheel_spin_velocity[i] * kConfig.wheel_radius_static) /
        safe_vehicle_speed;
    controller->wheel_slip_acceleration[i] =
        (next_slip - controller->wheel_slip[i]) / controller->delta_time;
    controller->wheel_slip[i] = next_slip;
  }

  controller->last_timestamp = input->timestamp;

  if (controller->vehicle_speed > kConfig.min_vehicle_velocity_threshold &&
      input->requested_pressure >
          (kConfig.min_pressure_threshold / kConfig.max_brake_pressure)) {
    for (int i = 0; i < ABS_WHEEL_COUNT; ++i) {
      if (controller->wheel_spin_velocity[i] * kConfig.wheel_radius_static >
          kConfig.min_wheel_velocity_threshold) {
        apply_phase(controller, i, input->requested_pressure);
        if (controller->ekf_is_active == 0) {
          abs_start_ekf(controller->vehicle_speed);
          controller->ekf_is_active = 1;
        }
      } else {
        controller->phase_states[i] = kPhaseOff;
        controller->max_wheel_slip[i] = kConfig.initial_max_wheel_slip;
      }
    }
  } else {
    for (int i = 0; i < ABS_WHEEL_COUNT; ++i) {
      controller->phase_states[i] = kPhaseOff;
      controller->max_wheel_slip[i] = kConfig.initial_max_wheel_slip;
      controller->brake_command[i] = input->requested_pressure;
    }
    controller->last_timestamp = 0.0f;
  }

  copy_debug_state(controller, output);
}
