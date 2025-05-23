//
// Created by lingwei on 4/3/24.
//

#ifndef MY_MUJOCO_SIMULATOR_HARDWARE_TYPES_H
#define MY_MUJOCO_SIMULATOR_HARDWARE_TYPES_H
#include <cstdint>

typedef struct USBCommand {
    float q_des_abad[4];
    float q_des_hip[4];
    float q_des_knee[4];
    float qd_des_abad[4];
    float qd_des_hip[4];
    float qd_des_knee[4];
    float kp_abad[4];
    float kp_hip[4];
    float kp_knee[4];
    float kd_abad[4];
    float kd_hip[4];
    float kd_knee[4];
    float tau_abad_ff[4];
    float tau_hip_ff[4];
    float tau_knee_ff[4];
    int32_t flags[2];
} USB_Command_t;

typedef struct USBData {
    float q_abad[4];
    float q_hip[4];
    float q_knee[4];
    float qd_abad[4];
    float qd_hip[4];
    float qd_knee[4];
    float tau_abad[4];
    float tau_hip[4];
    float tau_knee[4];
    int32_t flags[2];
} USB_Data_t;

typedef struct IMUData {
    float q[4];
    float gyro[3];
    float accel[3];
} USB_Imu_t;

#endif //MY_MUJOCO_SIMULATOR_HARDWARE_TYPES_H
