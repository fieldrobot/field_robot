#ifndef DEFS_H
#define DEFS_H

#define SERIAL_BAUD         115200      // [-] Baud rate for built-in Serial (used for the Serial Monitor)interval

//general values
double throttle = 0;; //unit not defined

//timer values
double current_time_microsec; //time is in microseconds
double old_time_microsec;
double dt_microsec;
double current_time_millisec; //time is in milliseconds
double old_time_millisec;
double dt_millisec;

//data from IMU, possibly filtered etc. (angle, angular speed, ...)
//double angle_offset;
double measured_angle; //°
double adjusted_angle; //° (filtered and angle-offset adjusted)
double adjusted_angle_old;
double measured_angular_speed; //unit not identifiable
double adjusted_angular_speed; //ggf.: °/s
double adjusted_angular_speed_old;
double angle_offset = 179.84;
int16_t accX, accY, accZ; //six values received from imu-sensor
int16_t gyroX, gyroY, gyroZ;
int16_t temp;

//speed control
double kp_throttle = 0.0025;
double ki_throttle = 1;//420;
double speed_error_cumulated = 0;
double speed_error = 0;
double speed_error_old = 0;
double speed_control;
double maximum_speed_control = 60;
double maximum_speed_error_cumulated = 1;
double speed_control_result;

//angle/balance control
double kp = 82.8;
double kd = 4;
double ki = 45;
double angle_error_cumulated;
double angle_error;
double angle_error_old;
double balance_control_result;
double balance_control_result_old;
double maximum_angle_error_cumulated = 600;

//serial communication with pc

//serial communication with hoverboard and commands for hoverboard
double speed_command;
double steer_command = 0;
double estimated_speed;
double estimated_speed_filtered;
double maximum_speed = 600;
double maximum_steer;

#endif
