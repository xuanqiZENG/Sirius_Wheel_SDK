//
// Created by lingwei on 3/26/24.
//

#ifndef PROJECT_RT_USB2CAN_H
#define PROJECT_RT_USB2CAN_H
#include <cstdint>
#include "libusb-1.0/libusb.h"
#include "lcm/lcm-cpp.hpp"
#include <mutex>
#include <fstream>
#include "beast_usb2can.h"
#include "leg_control_command_lcmt.hpp"
#include "leg_control_data_lcmt.hpp"
#include "PeriodicTask.h"
#include <lcm/lcm-cpp.hpp>
#include "orientation_tools.h"
#include "rt_usb_imu.h"
#include <chrono>
#include "hardware_types.h"
#include "cppTypes.h"
#include "rt_rc_interface.h"

using namespace ori;
constexpr uint8_t NUMBER_CHIPS = 3;
constexpr uint16_t usb_motors_in_length = 376;
constexpr uint16_t usb_motors_out_length = 376;
constexpr uint16_t usb_motors_in_check_length = usb_motors_in_length / 4 - 1;
constexpr uint16_t usb_motors_out_check_length = usb_motors_out_length / 4 - 1;

const uint16_t imu_vendor_id = 0x1234;
const uint16_t imu_product_id = 0x6789;
const unsigned char endpoint_1 = 0x81;
const int IMU_RX_WORDS_PER_MESSAGE = 44;

#define KNEE_OFFSET_POS (-3.6114f + 0.036f) //note pos_offset from the motor perspective
#define HIP_OFFSET_POS (1.0275f - 0.2f)
#define ABAD_OFFSET_POS (-0.4802f - 0.236f)
const float leg_side_sign[12] = {1.f, 1.f, 10.0 / 14.0f, 
                           1.f, -1.f, -10.0 / 14.0f, 
                          -1.f, 1.f, 10.0 / 14.0f, 
                          -1.f, -1.f, -10.0 / 14.0f};
const float whl_side_sign[4] = {-1.f, 1.f, -1.f, 1.f};
const float leg_offset[12] = {-ABAD_OFFSET_POS, HIP_OFFSET_POS, KNEE_OFFSET_POS, 
                        ABAD_OFFSET_POS, -HIP_OFFSET_POS, -KNEE_OFFSET_POS,
                        ABAD_OFFSET_POS, HIP_OFFSET_POS, KNEE_OFFSET_POS,
                        -ABAD_OFFSET_POS, -HIP_OFFSET_POS, -KNEE_OFFSET_POS};
const float whl_offset[4] = {0.0f, 0.0f, 0.0f, 0.0f};

#define P_LEFT_1_MIN (-3.93f) // abad
#define P_LEFT_1_MAX (3.93f)  
#define P_RIGHT_2_MIN (-3.93f)  // hip
#define P_RIGHT_2_MAX (3.93f)
#define P_KNEE_MIN (-2.269f * 28)
#define P_KNEE_MAX (2.269f * 28)

#define V_MIN (-15.0f)
#define V_MAX 15.0f

#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f

#define KI_MIN (-150.0f)
#define KI_MAX (150.0f)
#define T_MIN (-25.0f)
#define T_MAX (25.0f)

typedef struct USB_CMD_Pack {
    float p_cmd_;
    float v_cmd_;
    float kp_;
    float kd_;
    float t_ff_;
} USB_CMD_PACK_T;

typedef struct USB_CHIP_CMD {
    USB_CMD_PACK_T cmd_pack[6];
    uint32_t chip_flag[1];
} USB_CHIP_CMD_T;

typedef struct USB_DATA_Pack {
    float p_data_;
    float v_data_;
    float t_data_;
    float uq_;
    float ud_;
} USB_DATA_PACK_T;

typedef struct USB_CHIP_DATA {
    USB_DATA_PACK_T data_pack[6];
    uint32_t chip_flag[1];
} USB_CHIP_DATA_T;

typedef struct USB_CMD {
    USB_CHIP_CMD_T cmds_[NUMBER_CHIPS];
    uint32_t checksum;
} USB_Cmd_T;

typedef union USB_CMD_UNION {
    USB_Cmd_T usb_cmd;
    uint8_t usb_cmd_buff[usb_motors_out_length];
} USB_Cmd_U;

typedef struct USB_DATA {
    USB_CHIP_DATA_T leg_data[NUMBER_CHIPS];
    uint32_t checksum;
} USB_Data_T;

typedef union USB_DATA_UNION {
    USB_Data_T usb_data;
    uint8_t usb_data_buff[usb_motors_in_length];
} USB_Data_U;

class Beast_USB2CAN {
public:
    explicit Beast_USB2CAN(uint16_t vendor_id, uint16_t product_id, uint8_t _motors_epin, uint8_t _motors_epout);

    ~Beast_USB2CAN();

    // data union of this class is a temp buff, data checkok, memcpy to controll databuff.
    void USB2CAN_SetBuffer(USB_Cmd_T *_control_cmd, USB_Data_T *_controller_data);

    void start_transfer();

    void motor_epin_callback(struct libusb_transfer *_transfer);

    void motor_epout_callback(struct libusb_transfer *_transfer);

    void lock_in_mutex();

    void lock_out_mutex();

    void unlock_out_mutex();

    void unlock_in_mutex();
    libusb_context *ctx{};
    void set_write_variable() {
        write_ = true;
    }

    void register_ctrl_func(void (*func)(void)) {
        control_func = func;
    }
    void register_read_func(void (*func)(void)) {
        read_func = func;
    }
    lcm::LCM Controller2Robot;
    lcm::LCM Robot2Controller;
    leg_control_command_lcmt controller_lowcmd_{};
    leg_control_data_lcmt robot_lowdata_{};
    void start_transfer_sync();
    void _thread_rec_run();
    void _thread_sen_run();
    void _thread_imu_run();
    void start_sub_thread();
    void handleController2RobotLCM(const lcm::ReceiveBuffer* rbuf,
                                      const std::string& chan,
                                      const leg_control_command_lcmt* msg);
    PeriodicTaskManager taskManager;
    PeriodicMemberFunction<Beast_USB2CAN> receiveTask;
    PeriodicMemberFunction<Beast_USB2CAN> sendTask;
    PeriodicMemberFunction<Beast_USB2CAN> imuTask;
    PeriodicMemberFunction<Beast_USB2CAN> gampadTask;
    std::unique_ptr<N_Communication::USB_COM_IMU> usb_imu;
    bool                     out_zero_flag;
    void joint_leg_control_motor();
    IMUData imu_data{},imu_compen{};
    int Normal_print_count=0;
    void publish_data2Controller();
    void imu_compensation(IMUData* source, IMUData* final);
    std::mutex mutex_imu_data;
    std::mutex mutex_cmd_data,mutex_real_data;
    void initialized_cmd_from_controller();
    inline double clampMinMax(double value, double min, double max) {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }

private:
    // for controller data protocals
    uint8_t motors_endpoint_in;
    uint8_t motors_endpoint_out;
    libusb_transfer *transfer_in_motors{};
    libusb_transfer *transfer_out_motors{};
    uint16_t usb_vendor_id;
    uint16_t usb_product_id;
    libusb_device_handle *device_handle{};
    int count_usb_send=0;
    bool first_run_in_lcmsend=false;

    lcm::LCM usb_cmd_LCM;
    lcm::LCM usb_data_LCM;
    std::mutex* usb_in_mutex;
    std::mutex* usb_out_mutex;
    USB_Data_U *usb_data_u{};
    USB_Cmd_U *usb_cmd_u{};

    USB_Cmd_T *control_cmd{};
    USB_Data_T *control_data{};
    int port_xbox=0;

    void thread_run_xbox();

    void Deal_Usb_In_Data();

    void Deal_Usb_Out_Cmd();
    int count_print=0;

    void (*control_func)(void);
    void (*read_func)(void);
    std::chrono::steady_clock::time_point time_last_in;
    std::chrono::steady_clock::time_point time_now_in;
    std::chrono::steady_clock::time_point time_last_out;
    std::chrono::steady_clock::time_point time_now_out;
    std::fstream read_file_;
    bool write_ = false;

    Mat3<float> rot_offset;
    Eigen::Vector3f eigen_gyro, eigen_acc;
    Eigen::Vector3f eigen_gyro_rotated, eigen_acc_rotated;
    Eigen::Vector4f eigen_quat, eigen_quat_rotated;
};

void usb_motors_in_cbf_wrapper(struct libusb_transfer *_transfer);

void usb_motors_out_cbf_wrapper(struct libusb_transfer *_transfer);
#endif //PROJECT_RT_USB2CAN_H
