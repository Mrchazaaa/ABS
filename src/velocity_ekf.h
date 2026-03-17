#ifndef ABS_LIBRARY_VELOCITY_EKF_H_
#define ABS_LIBRARY_VELOCITY_EKF_H_

int abs_start_ekf(double initial_velocity);
double abs_step_ekf(
    double driving_angle,
    double front_wheel_vel,
    double rear_wheel_vel);

#endif  // ABS_LIBRARY_VELOCITY_EKF_H_
