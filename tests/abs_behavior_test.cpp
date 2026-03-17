#include "ABS/abs.h"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

constexpr float kFastWheelSpinVelocity = 40.0f;
constexpr float kSlowWheelSpinVelocity = 5.0f;
constexpr float kActivePressure = 0.5f;
constexpr float kLowerActivePressure = 0.35f;
constexpr float kInactivePressure = 0.0005f;
constexpr float kTimeStep = 0.1f;
constexpr float kFloatTolerance = 1e-5f;

bool nearly_equal(float actual, float expected, float tolerance = kFloatTolerance)
{
  return std::fabs(actual - expected) <= tolerance;
}

void require(bool condition, const char *message)
{
  if (!condition) {
    std::cerr << message << '\n';
    std::exit(1);
  }
}

AbsStepInput make_input(
    float timestamp,
    float requested_pressure,
    float fl,
    float fr,
    float rl,
    float rr)
{
  AbsStepInput input{};
  input.timestamp = timestamp;
  input.requested_pressure = requested_pressure;
  input.wheel_spin_velocity[ABS_FL] = fl;
  input.wheel_spin_velocity[ABS_FR] = fr;
  input.wheel_spin_velocity[ABS_RL] = rl;
  input.wheel_spin_velocity[ABS_RR] = rr;
  return input;
}

void require_all_wheels_equal(
    const float values[ABS_WHEEL_COUNT],
    float expected,
    const char *message)
{
  for (int i = 0; i < ABS_WHEEL_COUNT; ++i) {
    require(nearly_equal(values[i], expected), message);
  }
}

void require_all_phases(
    const int values[ABS_WHEEL_COUNT],
    int expected,
    const char *message)
{
  for (int i = 0; i < ABS_WHEEL_COUNT; ++i) {
    require(values[i] == expected, message);
  }
}

void test_passthrough_when_vehicle_speed_is_below_threshold()
{
  AbsController *controller = abs_create();
  AbsStepOutput output{};

  const AbsStepInput input = make_input(
      1.0f,
      kActivePressure,
      kSlowWheelSpinVelocity,
      kSlowWheelSpinVelocity,
      kSlowWheelSpinVelocity,
      kSlowWheelSpinVelocity);

  abs_step(controller, &input, &output);

  require_all_wheels_equal(
      output.brake_command,
      kActivePressure,
      "Low-speed input should pass requested pressure through unchanged.");
  require_all_phases(
      output.debug.phase_states,
      0,
      "Low-speed input should keep all wheels out of ABS.");

  const AbsConfig *config = abs_get_config();
  require_all_wheels_equal(
      output.debug.max_wheel_slip,
      config->initial_max_wheel_slip,
      "Low-speed input should restore the initial max wheel slip.");

  abs_destroy(controller);
}

void test_passthrough_when_requested_pressure_is_below_threshold()
{
  AbsController *controller = abs_create();
  AbsStepOutput output{};

  const AbsStepInput input = make_input(
      1.0f,
      kInactivePressure,
      kFastWheelSpinVelocity,
      kFastWheelSpinVelocity,
      kFastWheelSpinVelocity,
      kFastWheelSpinVelocity);

  abs_step(controller, &input, &output);

  require_all_wheels_equal(
      output.brake_command,
      kInactivePressure,
      "Pressure below the ABS threshold should pass through unchanged.");
  require_all_phases(
      output.debug.phase_states,
      0,
      "Pressure below the ABS threshold should keep ABS inactive.");

  abs_destroy(controller);
}

void test_abs_enters_apply_phase_when_speed_and_pressure_are_high_enough()
{
  AbsController *controller = abs_create();
  AbsStepOutput output{};

  const AbsStepInput input = make_input(
      1.0f,
      kActivePressure,
      kFastWheelSpinVelocity,
      kFastWheelSpinVelocity,
      kFastWheelSpinVelocity,
      kFastWheelSpinVelocity);

  abs_step(controller, &input, &output);

  require_all_phases(
      output.debug.phase_states,
      1,
      "High-speed braking should move all wheels into the initial ABS apply phase.");
  require_all_wheels_equal(
      output.brake_command,
      0.0f,
      "The first ABS activation step should only arm the apply phase.");

  abs_destroy(controller);
}

void test_phase_one_tracks_requested_pressure_on_following_step()
{
  AbsController *controller = abs_create();
  AbsStepOutput output{};

  AbsStepInput input = make_input(
      1.0f,
      kActivePressure,
      kFastWheelSpinVelocity,
      kFastWheelSpinVelocity,
      kFastWheelSpinVelocity,
      kFastWheelSpinVelocity);
  abs_step(controller, &input, &output);

  input.timestamp += kTimeStep;
  input.requested_pressure = kLowerActivePressure;
  abs_step(controller, &input, &output);

  require_all_phases(
      output.debug.phase_states,
      1,
      "With steady wheel speeds, the controller should remain in phase one.");
  require_all_wheels_equal(
      output.brake_command,
      kLowerActivePressure,
      "In phase one, brake output should follow the requested pressure.");
  require(nearly_equal(output.debug.delta_time, kTimeStep),
      "The second step should use the elapsed timestamp as delta time.");

  abs_destroy(controller);
}

void test_only_fast_wheels_enter_abs_when_other_wheels_are_below_wheel_threshold()
{
  AbsController *controller = abs_create();
  AbsStepOutput output{};

  const AbsStepInput input = make_input(
      1.0f,
      kActivePressure,
      kFastWheelSpinVelocity,
      kFastWheelSpinVelocity,
      kSlowWheelSpinVelocity,
      kSlowWheelSpinVelocity);

  abs_step(controller, &input, &output);

  require(output.debug.phase_states[ABS_FL] == 1,
      "The front-left wheel should enter ABS when it exceeds the wheel-speed threshold.");
  require(output.debug.phase_states[ABS_FR] == 1,
      "The front-right wheel should enter ABS when it exceeds the wheel-speed threshold.");
  require(output.debug.phase_states[ABS_RL] == 0,
      "The rear-left wheel should remain out of ABS below the wheel-speed threshold.");
  require(output.debug.phase_states[ABS_RR] == 0,
      "The rear-right wheel should remain out of ABS below the wheel-speed threshold.");

  abs_destroy(controller);
}

void test_deactivation_resets_state_and_restores_passthrough_output()
{
  AbsController *controller = abs_create();
  AbsStepOutput output{};

  AbsStepInput input = make_input(
      1.0f,
      kActivePressure,
      kFastWheelSpinVelocity,
      kFastWheelSpinVelocity,
      kFastWheelSpinVelocity,
      kFastWheelSpinVelocity);
  abs_step(controller, &input, &output);

  input.timestamp += kTimeStep;
  input.requested_pressure = kInactivePressure;
  abs_step(controller, &input, &output);

  require_all_phases(
      output.debug.phase_states,
      0,
      "Dropping below the pressure threshold should reset all ABS phase state.");
  require_all_wheels_equal(
      output.brake_command,
      kInactivePressure,
      "After ABS deactivates, brake output should return to passthrough.");

  input.timestamp += kTimeStep;
  input.requested_pressure = kActivePressure;
  abs_step(controller, &input, &output);

  require_all_phases(
      output.debug.phase_states,
      1,
      "A new active braking event should restart ABS from phase one.");
  require(nearly_equal(output.debug.delta_time, 1e-6f, 1e-7f),
      "After deactivation resets timing, the next activation should start with the minimum delta time.");

  abs_destroy(controller);
}

void test_abs_reset_restores_initial_runtime_state()
{
  AbsController *controller = abs_create();
  AbsStepOutput output{};

  AbsStepInput input = make_input(
      1.0f,
      kActivePressure,
      kFastWheelSpinVelocity,
      kFastWheelSpinVelocity,
      kFastWheelSpinVelocity,
      kFastWheelSpinVelocity);
  abs_step(controller, &input, &output);

  abs_reset(controller);

  input = make_input(
      2.0f,
      kInactivePressure,
      kSlowWheelSpinVelocity,
      kSlowWheelSpinVelocity,
      kSlowWheelSpinVelocity,
      kSlowWheelSpinVelocity);
  abs_step(controller, &input, &output);

  require_all_phases(
      output.debug.phase_states,
      0,
      "abs_reset should clear all ABS phases.");
  require_all_wheels_equal(
      output.brake_command,
      kInactivePressure,
      "abs_reset should restore passthrough behavior on the next inactive step.");
  require(nearly_equal(output.debug.vehicle_speed,
      kSlowWheelSpinVelocity * abs_get_config()->wheel_radius_static),
      "After reset, vehicle speed should be recomputed from wheel speed on the next step.");

  abs_destroy(controller);
}

}  // namespace

int main()
{
  test_passthrough_when_vehicle_speed_is_below_threshold();
  test_passthrough_when_requested_pressure_is_below_threshold();
  test_abs_enters_apply_phase_when_speed_and_pressure_are_high_enough();
  test_phase_one_tracks_requested_pressure_on_following_step();
  test_only_fast_wheels_enter_abs_when_other_wheels_are_below_wheel_threshold();
  test_deactivation_resets_state_and_restores_passthrough_output();
  test_abs_reset_restores_initial_runtime_state();
  return 0;
}
