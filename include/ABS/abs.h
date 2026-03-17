#ifndef ABS_LIBRARY_ABS_H_
#define ABS_LIBRARY_ABS_H_

#ifdef __cplusplus
extern "C" {
#endif

#define ABS_WHEEL_COUNT 4

enum {
  ABS_FL = 0,
  ABS_FR = 1,
  ABS_RL = 2,
  ABS_RR = 3,
};

typedef struct AbsController AbsController;

typedef struct {
  float min_vehicle_velocity_threshold;
  float min_wheel_velocity_threshold;
  float min_pressure_threshold;
  float apply_delay;
  float primary_apply_rate;
  float secondary_apply_rate;
  float release_rate;
  float min_wheel_spin_acceleration;
  float max_wheel_spin_acceleration;
  float initial_max_wheel_slip;
  float max_brake_pressure;
  float wheel_radius_static;
} AbsConfig;

typedef struct {
  int phase_states[ABS_WHEEL_COUNT];
  float wheel_spin_velocity[ABS_WHEEL_COUNT];
  float wheel_spin_acceleration[ABS_WHEEL_COUNT];
  float wheel_slip_acceleration[ABS_WHEEL_COUNT];
  float wheel_slip[ABS_WHEEL_COUNT];
  float max_wheel_slip[ABS_WHEEL_COUNT];
  float delta_time;
  float vehicle_speed;
} AbsDebugState;

typedef struct {
  float timestamp;
  float requested_pressure;
  float wheel_spin_velocity[ABS_WHEEL_COUNT];
} AbsStepInput;

typedef struct {
  float brake_command[ABS_WHEEL_COUNT];
  AbsDebugState debug;
} AbsStepOutput;

const AbsConfig *abs_get_config(void);
AbsController *abs_create(void);
void abs_destroy(AbsController *controller);
void abs_reset(AbsController *controller);
void abs_step(
    AbsController *controller,
    const AbsStepInput *input,
    AbsStepOutput *output);

#ifdef __cplusplus
}
#endif

#endif  // ABS_LIBRARY_ABS_H_
