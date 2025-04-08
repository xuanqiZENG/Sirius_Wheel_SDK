#include <iostream>
// #include "include/usb_interface.h"
#include <memory>
#include <ctime>
#include <cmath>
#include <time.h>
#include <sys/time.h>
#include <sys/select.h>
#include <thread>
#include "include/rt_rc_interface.h"
#include "beast_usb2can.h"

#include <lcm/lcm-cpp.hpp>
const uint16_t sirius_usb2can_vendor_id = 0x1111;
const uint16_t sirius_usb2can_product_id = 0x2222;
const uint8_t  motors_ep_in = 0x81;
const uint8_t  motors_ep_out = 0x01;
// spi_command_t * usb_cmd_from_controller;
// spi_data_t * usb_data_to_controller;
USB_Cmd_T *usb_cmd_from_controller;
USB_Data_T *usb_data_to_controller;

uint8_t first_run = 1;
static int counter = 0;
extern rc_control_settings rc_control;


/**/
int main() {
    usb_cmd_from_controller = new USB_Cmd_T();
    usb_data_to_controller = new USB_Data_T();
    int complete = 0;
    struct timeval timestru{};
    timestru.tv_sec = 0;
    timestru.tv_usec = 1000;

    // Sirius_USB2CAN_Board spUSB2CAN(sirius_usb2can_vendor_id,//
    //                                sirius_usb2can_product_id,
    //                                motors_ep_in,
    //                                motors_ep_out);
    // spUSB2CAN.USB2CAN_SetBuffer(usb_cmd_from_controller, usb_data_to_controller);
    // spUSB2CAN.USB2CAN_Start_Transfer_Ans();
    // joystick
    // int port_=init_xbox_js();
    // std::thread th1(run_xbox,port_);

    Beast_USB2CAN spUSB2CAN(sirius_usb2can_vendor_id, 
                                   sirius_usb2can_product_id,
                                   motors_ep_in,
                                   motors_ep_out);
    spUSB2CAN.USB2CAN_SetBuffer(usb_cmd_from_controller, usb_data_to_controller);
    spUSB2CAN.start_transfer();
    spUSB2CAN.set_write_variable();

    // IMU data publish use lcm
    while (1)
    {

        libusb_handle_events_timeout_completed(spUSB2CAN.ctx, &timestru, &complete);
        if (spUSB2CAN.out_zero_flag)
        {
            break;
        }
        
        // libusb_handle_events(spUSB2CAN.ctx);
    }
    // th1.join();

}

// void disable_motors(){
//     counter++;
//     float temp_angle = 0.001f * (float)counter;
//     for(int i = 0; i < (2 * NUMBER_CHIPS); i++)
//     {
//         //usb_cmd_from_controller->q_des_abad[i] = 1.5;
//         // usb_cmd_from_controller->q_des_hip[i] = 2;
//         // usb_cmd_from_controller->q_des_knee[i] = 3;
//         usb_cmd_from_controller->qd_des_abad[i] = 0;
// //        usb_cmd_from_controller->tau_abad_ff[i] = -90.f;
//         //usb_cmd_from_controller->kd_abad[i] = 1.f;
//        usb_cmd_from_controller->qd_des_hip[i]= 0;
// //        usb_cmd_from_controller->kd_hip[i] = 1.f;
// //        usb_cmd_from_controller->qd_des_knee[i]= 5;
//         // usb_cmd_from_controller->kp_abad[i]= usb_cmd_from_controller->kp_hip[i] = usb_cmd_from_controller->kp_knee[i] = 10;
//         usb_cmd_from_controller->kd_abad[i]= usb_cmd_from_controller->kd_hip[i] = usb_cmd_from_controller->kd_knee[i] = 0;
//         //usb_cmd_from_controller->tau_abad_ff[i]= sinf(temp_angle) + 8;
// //        usb_cmd_from_controller->tau_hip_ff[i]= 9;
// //        usb_cmd_from_controller->tau_knee_ff[i]= 10;
//         // 00000 100 01 Position / VEL / Mit

//     usb_cmd_from_controller->flags[0] = usb_cmd_from_controller->flags[1] = (0x01 << 1); //disable
//     }
// };
// void fresh_usb_cmd_from_controller()
// {
    
//     float temp_angle = 0.1f * (float)counter;
//     for(int i = 0; i < (2 * NUMBER_CHIPS); i++)
//     {
//         //usb_cmd_from_controller->q_des_abad[i] = 1.5;
//         // usb_cmd_from_controller->q_des_hip[i] = 2;
//         // usb_cmd_from_controller->q_des_knee[i] = 3;
//         usb_cmd_from_controller->qd_des_abad[i] = 0.0f;//1.f* sinf(temp_angle);
// //        usb_cmd_from_controller->tau_abad_ff[i] = -90.f;
//         //usb_cmd_from_controller->kd_abad[i] = 1.f;
//        usb_cmd_from_controller->qd_des_hip[i]= 0.0f;//1.f* sinf(temp_angle);
// //        usb_cmd_from_controller->kd_hip[i] = 1.f;
//        usb_cmd_from_controller->qd_des_knee[i]= 0.0f;//1.f* sinf(temp_angle);
//         // usb_cmd_from_controller->kp_abad[i]= usb_cmd_from_controller->kp_hip[i] = usb_cmd_from_controller->kp_knee[i] = 10;
//         usb_cmd_from_controller->kd_abad[i]= usb_cmd_from_controller->kd_hip[i] = usb_cmd_from_controller->kd_knee[i] = 0;
//         //usb_cmd_from_controller->tau_abad_ff[i]= sinf(temp_angle) + 8;
// //        usb_cmd_from_controller->tau_hip_ff[i]= 9;
// //        usb_cmd_from_controller->tau_knee_ff[i]= 10;
//         // 00000 100 01 Position / VEL / Mit
//         // 右边第一位是使能
//         // 第二个是disable
//         // 第三个就是MIT mode
//         // 第四个是Vel Mode
//         // 101 MIT mode // 高位在前
//         // 1001 Vel mode
//         // 
//         if(first_run)
//         {
//             usb_cmd_from_controller->flags[0] = usb_cmd_from_controller->flags[1]
//                     = usb_cmd_from_controller->flags[2] = (0x01 << 1); //disable
//             //usb_cmd_from_controller->flags[i] = 0;
//         } else {
//             //usb_cmd_from_controller->flags[i] = 1;
//             usb_cmd_from_controller->flags[0] = usb_cmd_from_controller->flags[1]
//             = usb_cmd_from_controller->flags[2]= (0x01 << 2) | 0x01;// vel
//         }
//     }
//     if(counter > 10) {
//         first_run = 0;
//     }
//     if (counter > 100)
//     {
//         counter=100;
//     }
//     counter++;
    
// }
