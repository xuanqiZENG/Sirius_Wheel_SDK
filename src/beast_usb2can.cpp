#include "beast_usb2can.h"
#include "iostream"
#include <cstdlib>
#include <thread>
#include <cstring>
// #include "orientation_tools.h"

extern rc_control_settings rc_control;

static uint32_t data_checksum(const uint32_t *data_to_check, uint32_t check_length) {
    uint32_t t = 0;
    for (int i = 0; i < check_length; i++) {
        t = t ^ data_to_check[i];
    }

    return t;
}

Beast_USB2CAN::Beast_USB2CAN(const uint16_t vendor_id, const uint16_t product_id, const uint8_t _motors_epin,
                             const uint8_t _motors_epout): motors_endpoint_in(_motors_epin), motors_endpoint_out(_motors_epout),
                                                     usb_vendor_id(vendor_id),
                                                     usb_product_id(product_id),
                                                     Controller2Robot("udpm://239.255.76.67:7667?ttl=255"),
                                                     Robot2Controller("udpm://239.255.76.67:7667?ttl=255"),
                                                     receiveTask(&taskManager, .001, "recv_task", &Beast_USB2CAN::_thread_rec_run, this),
                                                     sendTask(&taskManager, .001, "send_task", &Beast_USB2CAN::_thread_sen_run, this),
                                                     imuTask(&taskManager, .001, "imu_task", &Beast_USB2CAN::_thread_imu_run, this),
                                                     gampadTask(&taskManager, .005, "gamepad_task", &Beast_USB2CAN::thread_run_xbox, this) {
    usb_cmd_u = new USB_Cmd_U();
    usb_data_u = new USB_Data_U();
    control_cmd = new USB_Cmd_T();
    control_data = new USB_Data_T();
    usb_in_mutex = new std::mutex();
    usb_out_mutex = new std::mutex();
    std::cout<<"123"<<std::endl;
    out_zero_flag = 0;

    time_last_out = std::chrono::steady_clock::now();
    time_now_out = std::chrono::steady_clock::now();

    time_last_in = std::chrono::steady_clock::now();
    time_now_in = std::chrono::steady_clock::now();
    libusb_init(&ctx);
    transfer_out_motors = libusb_alloc_transfer(0);
    transfer_in_motors = libusb_alloc_transfer(0);
    device_handle = libusb_open_device_with_vid_pid(ctx, usb_vendor_id, usb_product_id);
    if (libusb_kernel_driver_active(device_handle, 0x00)) {
        int success = libusb_detach_kernel_driver(device_handle, 0x00);
        if (success != 0) {
            std::cerr << "Detach Driver Failed!" << std::endl;
            exit(EXIT_FAILURE);
        }
    }
    int claim_interface = libusb_claim_interface(device_handle, 0x00);
    if (claim_interface != 0) {
        std::cerr << "Claim Driver Failed!" << std::endl;
        exit(EXIT_FAILURE);
    } else {
        std::cout << "Claim USB Device OK!\n";
    }

    // initiate files
    this->read_file_.open("motor_data.txt", std::ios::out | std::ios::trunc);
    rot_offset = coordinateRotation(CoordinateAxis::Z, -1.5707f)*coordinateRotation(CoordinateAxis::Y,0.0f)*coordinateRotation(CoordinateAxis::X,0.0f);
    usb_imu = std::make_unique<N_Communication::USB_COM_IMU>(IMU_RX_WORDS_PER_MESSAGE, 4,
                                                                  N_Communication::Vendor_id_Hex(imu_vendor_id),
                                                                  N_Communication::Product_id_Hex(imu_product_id),
                                                                  endpoint_1, 0x0);
    try
    {
        port_xbox=init_xbox_js();
        std::cout<<"port initialized ok!"<<std::endl;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what()<<" port xbox initialized failed" << '\n';
    }

    start_sub_thread();
    initialized_cmd_from_controller();
}

Beast_USB2CAN::~Beast_USB2CAN() {
    this->read_file_.close();
    delete usb_cmd_u;
    delete usb_data_u;
    delete control_cmd;
    delete control_data;
    delete  usb_in_mutex;
    delete  usb_out_mutex;

    libusb_free_transfer(transfer_in_motors);
    libusb_free_transfer(transfer_out_motors);
    libusb_release_interface(device_handle, 0);
    libusb_close(device_handle);
    libusb_exit(ctx);
}

void Beast_USB2CAN::initialized_cmd_from_controller(){
    for (int i = 0; i < 16; i++)
    {
        controller_lowcmd_.q_des[i] = 0.0;
        controller_lowcmd_.q_des[i] = 0.0;
        controller_lowcmd_.q_des[i] = 0.0;
        controller_lowcmd_.qd_des[i] =0.0;
        controller_lowcmd_.qd_des[i] =0.0;
        controller_lowcmd_.qd_des[i] =0.0;
        controller_lowcmd_.kp_joint[i] = 0.0;
        controller_lowcmd_.kp_joint[i] = 0.0;
        controller_lowcmd_.kp_joint[i] = 0.0;
        controller_lowcmd_.kd_joint[i] = 0.0;
        controller_lowcmd_.kd_joint[i] = 0.0;
        controller_lowcmd_.kd_joint[i] = 0.0;
        controller_lowcmd_.tau_ff[i] = 0.0;
        controller_lowcmd_.tau_ff[i] = 0.0;
        controller_lowcmd_.tau_ff[i] = 0.0;
        controller_lowcmd_.tau_des[i] = 0.0;
        controller_lowcmd_.tau_des[i] = 0.0;
        controller_lowcmd_.tau_des[i] = 0.0;
    }

}

void Beast_USB2CAN::USB2CAN_SetBuffer(USB_Cmd_T *_control_cmd, USB_Data_T *_controller_data) {
    control_cmd = _control_cmd;
    control_data = _controller_data;
}

void Beast_USB2CAN::motor_epin_callback(struct libusb_transfer *_transfer) {
    if (_transfer->status != LIBUSB_TRANSFER_COMPLETED) {
        std::cout << "Motor Ep81 IN Error! Transfer again!\n";
    } else if (_transfer->status == LIBUSB_TRANSFER_COMPLETED) {
        // this->read_func();
        this->Deal_Usb_In_Data();
        
        time_now_in = std::chrono::steady_clock::now();
        std::chrono::duration<double, std::micro> time_used = std::chrono::duration_cast<std::chrono::duration<double,
            std::micro> >(time_now_in - time_last_in);
        // std::cout << "[Time Interval In]: " << time_used.count() << " us\n";
        time_last_in = time_now_in;
        libusb_submit_transfer(_transfer);
    }
    // std::cout << _transfer->status << "\n";
}

void Beast_USB2CAN::motor_epout_callback(struct libusb_transfer *_transfer) {
    if (_transfer->status != LIBUSB_TRANSFER_COMPLETED) {
        std::cout << "Motor Ep01 OUT Error! Transfer again!\n";
    } else if (_transfer->status == LIBUSB_TRANSFER_COMPLETED) {
        // this->control_func();
        // std::cout<<rc_control.mode<<std::endl;
        if (rc_control.mode==RC_mode::OFF)
        {
            mutex_cmd_data.lock();
            control_cmd->cmds_[0].chip_flag[0] = (0x01 << 1);
            control_cmd->cmds_[1].chip_flag[0] = (0x01 << 1);
            control_cmd->cmds_[2].chip_flag[0] = (0x01 << 1);
            mutex_cmd_data.unlock();
            if (count_print%1000==0)
            {
                std::cout<<"[Normal] In Passive mode!!---"<<std::endl;
            }
        }
        else if(rc_control.mode==RC_mode::LOWLEVELSDK)
        {
            if (count_print%1000==0){
            std::cout<<"In Controller mode!!---"<<std::endl;
            }
            joint_leg_control_motor();
        }

        count_print++;
        if (count_print>1001)
        {
        count_print=0;
        }

        this->Deal_Usb_Out_Cmd();
        time_now_out = std::chrono::steady_clock::now();
        std::chrono::duration<double, std::micro> time_used = std::chrono::duration_cast<std::chrono::duration<double,
            std::micro> >(time_now_out - time_last_out);
        // std::cout << "[Time Interval Out]: " << time_used.count() << " us\n";
        time_last_out = time_now_out;

        libusb_submit_transfer(_transfer);
    }
}


void Beast_USB2CAN::Deal_Usb_In_Data() {
    const uint32_t t = data_checksum(reinterpret_cast<uint32_t *>(usb_data_u), usb_motors_in_check_length);
    volatile uint8_t leg_id = 0;
    volatile uint8_t data_index = 0;
    if (usb_data_u->usb_data.checksum == t) {
        memcpy(control_data, usb_data_u, sizeof(USB_Data_T));
        if (write_) {
        }
    } else {
        std::cout << "usb data checksum error\n";
    }
}

void Beast_USB2CAN::Deal_Usb_Out_Cmd() {
    memcpy(usb_cmd_u, control_cmd, sizeof(USB_Cmd_T));
    // std::cout << usb_cmd_u->usb_cmd.cmds_[0].chip_flag[0] << "\n";
    usb_cmd_u->usb_cmd.checksum = data_checksum(reinterpret_cast<uint32_t *>(usb_cmd_u), usb_motors_out_check_length);
}

void Beast_USB2CAN::start_transfer() {
    libusb_fill_interrupt_transfer(transfer_out_motors, device_handle, motors_endpoint_out,
                                   usb_cmd_u->usb_cmd_buff, usb_motors_out_length, usb_motors_out_cbf_wrapper, this, 0);
    libusb_fill_interrupt_transfer(transfer_in_motors, device_handle, motors_endpoint_in, usb_data_u->usb_data_buff,
                                   usb_motors_in_length, usb_motors_in_cbf_wrapper, this, 0);
    libusb_submit_transfer(transfer_in_motors);
    libusb_submit_transfer(transfer_out_motors);
    if ((!transfer_out_motors->status) & (!transfer_in_motors->status)) {
        std::cout << "[Good] All endpoints start transfering!\n";
    } else {
        std::cout << "[Bad] Some endpoints not work\n";
        exit(EXIT_FAILURE);
    }
}

void Beast_USB2CAN::lock_in_mutex() {
    usb_in_mutex->lock();
}

void Beast_USB2CAN::lock_out_mutex() {
    usb_out_mutex->lock();
}

void Beast_USB2CAN::unlock_out_mutex() {
    usb_out_mutex->unlock();
}

void Beast_USB2CAN::unlock_in_mutex() {
    usb_in_mutex->unlock();
}

void usb_motors_in_cbf_wrapper(struct libusb_transfer *_transfer) {
    auto *temp = reinterpret_cast<Beast_USB2CAN *>(_transfer->user_data);
    temp->motor_epin_callback(_transfer);
}

void usb_motors_out_cbf_wrapper(struct libusb_transfer *_transfer) {
    auto *temp = reinterpret_cast<Beast_USB2CAN *>(_transfer->user_data);
    temp->motor_epout_callback(_transfer);
}

void Beast_USB2CAN::start_transfer_sync()
{
    this->Deal_Usb_Out_Cmd();
    int actual_length = 0;
    bool success = libusb_interrupt_transfer(device_handle, motors_endpoint_out, usb_cmd_u->usb_cmd_buff, usb_motors_out_length, &actual_length, 500);
    if(!success)
    {
        std::cout << "\n[Sych Transmit Success]\n";
    }
    else
    {
        std::cout << "\n[Transmit Error, Kill code and Try Again!] !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n";
    }

}

void Beast_USB2CAN::_thread_rec_run(){
    Controller2Robot.handle();
    
}

void Beast_USB2CAN::handleController2RobotLCM(const lcm::ReceiveBuffer* rbuf,
                                      const std::string& chan,
                                      const leg_control_command_lcmt* msg){
    (void)rbuf;
    (void)chan;
    memcpy(&controller_lowcmd_, msg, sizeof(controller_lowcmd_));
}

void Beast_USB2CAN::start_sub_thread(){
    Controller2Robot.subscribe("controller2robot", &Beast_USB2CAN::handleController2RobotLCM, this);
    receiveTask.start();
    sendTask.start();
    imuTask.start();
    gampadTask.start();
    std::cout<<"LCM Initialized!!"<<std::endl;
}

void Beast_USB2CAN::_thread_imu_run(){
    usb_imu->usb_imu_set_rx_buffer(&imu_data);
    usb_imu->USB_Com_Start_Trans_Asy();
    while (true) {
        libusb_handle_events(usb_imu->get_usb_ctx());
        // for lcm data 
//        mutex_imu_data.lock();
//        imu_data.accelerometer[0] = local_usb_drv->accel[0];
//        imu_data.accelerometer[1] = local_usb_drv->accel[1];
//        imu_data.accelerometer[2] = local_usb_drv->accel[2];
//        imu_data.quat[0] = local_usb_drv->q[0];//w
//        imu_data.quat[1] = local_usb_drv->q[1];//w
//        imu_data.quat[2] = local_usb_drv->q[2];//w
//        imu_data.quat[3] = local_usb_drv->q[3];//w
//        imu_data.gyro[0] = local_usb_drv->gyro[0]; //omega
//        imu_data.gyro[1] = local_usb_drv->gyro[1];
//        imu_data.gyro[2] = local_usb_drv->gyro[2];
//        mutex_imu_data.unlock();
    }
}

void Beast_USB2CAN::publish_data2Controller(){
 if (control_data  != nullptr)
 {
    mutex_real_data.lock();
    for (int i = 0; i < 6; i++)
    {   
        robot_lowdata_.q[i]  = (control_data->leg_data[i/6].data_pack[i-6*(i/6)].p_data_ - leg_offset[i]) * leg_side_sign[i];
        robot_lowdata_.qd[i]  = control_data->leg_data[i/6].data_pack[i-6*(i/6)].v_data_  * leg_side_sign[i];
        robot_lowdata_.tauEst[i]  = control_data->leg_data[i/6].data_pack[i-6*(i/6)].t_data_  / leg_side_sign[i];
        robot_lowdata_.uq[i]  = control_data->leg_data[i/6].data_pack[i-6*(i/6)].uq_;
        robot_lowdata_.ud[i]  = control_data->leg_data[i/6].data_pack[i-6*(i/6)].ud_;
    }
        robot_lowdata_.q[6]  = (control_data->leg_data[1].data_pack[0].p_data_ - whl_offset[0]) * whl_side_sign[0];
        robot_lowdata_.qd[6]  = control_data->leg_data[1].data_pack[0].v_data_ * whl_side_sign[0];
        robot_lowdata_.tauEst[6]  = control_data->leg_data[1].data_pack[0].t_data_ / whl_side_sign[0];
        robot_lowdata_.uq[6]  = control_data->leg_data[1].data_pack[0].uq_;
        robot_lowdata_.ud[6]  = control_data->leg_data[1].data_pack[0].ud_;

        robot_lowdata_.q[7]  = (control_data->leg_data[1].data_pack[1].p_data_ - whl_offset[1]) * whl_side_sign[1];
        robot_lowdata_.qd[7]  = control_data->leg_data[1].data_pack[1].v_data_ * whl_side_sign[1];
        robot_lowdata_.tauEst[7]  = control_data->leg_data[1].data_pack[1].t_data_ / whl_side_sign[1];
        robot_lowdata_.uq[7]  = control_data->leg_data[1].data_pack[1].uq_;
        robot_lowdata_.ud[7]  = control_data->leg_data[1].data_pack[1].ud_;

    for (int i = 9; i < 15; i++)
    {   
        robot_lowdata_.q[i-1]  = (control_data->leg_data[i/6].data_pack[i-6*(i/6)].p_data_ - leg_offset[i-3]) * leg_side_sign[i-3];
        robot_lowdata_.qd[i-1]  = control_data->leg_data[i/6].data_pack[i-6*(i/6)].v_data_  * leg_side_sign[i-3];
        robot_lowdata_.tauEst[i-1]  = control_data->leg_data[i/6].data_pack[i-6*(i/6)].t_data_  / leg_side_sign[i-3];
        robot_lowdata_.uq[i-1]  = control_data->leg_data[i/6].data_pack[i-6*(i/6)].uq_;
        robot_lowdata_.ud[i-1]  = control_data->leg_data[i/6].data_pack[i-6*(i/6)].ud_;
    }

        robot_lowdata_.q[14]  = (control_data->leg_data[2].data_pack[3].p_data_ - whl_offset[2]) * whl_side_sign[2];
        robot_lowdata_.qd[14]  = control_data->leg_data[2].data_pack[3].v_data_ * whl_side_sign[2];
        robot_lowdata_.tauEst[14]  = control_data->leg_data[2].data_pack[3].t_data_ / whl_side_sign[2];
        robot_lowdata_.uq[14]  = control_data->leg_data[2].data_pack[3].uq_;
        robot_lowdata_.ud[14]  = control_data->leg_data[2].data_pack[3].ud_;

        robot_lowdata_.q[15]  = (control_data->leg_data[2].data_pack[4].p_data_ - whl_offset[3]) * whl_side_sign[3];
        robot_lowdata_.qd[15]  = control_data->leg_data[2].data_pack[4].v_data_ * whl_side_sign[3];
        robot_lowdata_.tauEst[15]  = control_data->leg_data[2].data_pack[4].t_data_ / whl_side_sign[3];
        robot_lowdata_.uq[15]  = control_data->leg_data[2].data_pack[4].uq_;
        robot_lowdata_.ud[15]  = control_data->leg_data[2].data_pack[4].ud_;

        // add imu2body transition.
        imu_compensation(&imu_data,&imu_compen);
        robot_lowdata_.accelerometer[0] = imu_compen.accel[0] ;
        robot_lowdata_.accelerometer[1] = imu_compen.accel[1] ;
        robot_lowdata_.accelerometer[2] = imu_compen.accel[2] ;
        robot_lowdata_.quat[0] = imu_compen.q[0] ;//w
        robot_lowdata_.quat[1] = imu_compen.q[1] ;//x
        robot_lowdata_.quat[2] = imu_compen.q[2] ;//y
        robot_lowdata_.quat[3] = imu_compen.q[3] ;//z
        robot_lowdata_.gyro[0] = imu_compen.gyro[0] ;// //omega
        robot_lowdata_.gyro[1] = imu_compen.gyro[1] ;//
        robot_lowdata_.gyro[2] = imu_compen.gyro[2] ;//
    Robot2Controller.publish("robot2controller",&robot_lowdata_);
    mutex_real_data.unlock();
 }
}

void Beast_USB2CAN::joint_leg_control_motor(){
    mutex_cmd_data.lock();
    for (int i = 0; i<16; i++){
        controller_lowcmd_.tau_des[i] = controller_lowcmd_.kp_joint[i] * (controller_lowcmd_.q_des[i] - robot_lowdata_.q[i]) +
                                       controller_lowcmd_.kd_joint[i] * (controller_lowcmd_.qd_des[i] - robot_lowdata_.qd[i]) 
                                       + controller_lowcmd_.tau_ff[i];
        controller_lowcmd_.tau_des[i] = clampMinMax(controller_lowcmd_.tau_des[i],T_MIN,T_MAX);
    }
    
    for (int i = 0; i < 6; i++)
    {
        // control_cmd->cmds_[i/6].cmd_pack[i-6*(i/6)].p_cmd_ = (controller_lowcmd_.q_des[i] / leg_side_sign[i]) + leg_offset[i];
        // control_cmd->cmds_[i/6].cmd_pack[i-6*(i/6)].v_cmd_ = controller_lowcmd_.qd_des[i] / leg_side_sign[i];
        // control_cmd->cmds_[i/6].cmd_pack[i-6*(i/6)].kp_ = controller_lowcmd_.kp_joint[i];
        // control_cmd->cmds_[i/6].cmd_pack[i-6*(i/6)].kd_ = controller_lowcmd_.kd_joint[i];
        // control_cmd->cmds_[i/6].cmd_pack[i-6*(i/6)].t_ff_ = controller_lowcmd_.tau_ff[i]* leg_side_sign[i]; 
        control_cmd->cmds_[i/6].cmd_pack[i-6*(i/6)].p_cmd_ = 0.0;
        control_cmd->cmds_[i/6].cmd_pack[i-6*(i/6)].v_cmd_ = 0.0;
        control_cmd->cmds_[i/6].cmd_pack[i-6*(i/6)].kp_ = 0.0;
        control_cmd->cmds_[i/6].cmd_pack[i-6*(i/6)].kd_ = 0.0;
        control_cmd->cmds_[i/6].cmd_pack[i-6*(i/6)].t_ff_ = controller_lowcmd_.tau_des[i]* leg_side_sign[i]; 
    }
    
    //    control_cmd->cmds_[1].cmd_pack[0].p_cmd_ = (controller_lowcmd_.q_des[6] / whl_side_sign[0]) + whl_offset[0];
    //    control_cmd->cmds_[1].cmd_pack[0].v_cmd_ = controller_lowcmd_.qd_des[6] / whl_side_sign[0];
    //    control_cmd->cmds_[1].cmd_pack[0].kp_ = controller_lowcmd_.kp_joint[6];
    //    control_cmd->cmds_[1].cmd_pack[0].kd_ = controller_lowcmd_.kd_joint[6];
    //    control_cmd->cmds_[1].cmd_pack[0].t_ff_ = controller_lowcmd_.tau_ff[6] * whl_side_sign[0];

    //    control_cmd->cmds_[1].cmd_pack[1].p_cmd_ = (controller_lowcmd_.q_des[7] / whl_side_sign[1]) + whl_offset[1];
    //    control_cmd->cmds_[1].cmd_pack[1].v_cmd_ = controller_lowcmd_.qd_des[7] / whl_side_sign[1];
    //    control_cmd->cmds_[1].cmd_pack[1].kp_ = controller_lowcmd_.kp_joint[7];
    //    control_cmd->cmds_[1].cmd_pack[1].kd_ = controller_lowcmd_.kd_joint[7];
    //    control_cmd->cmds_[1].cmd_pack[1].t_ff_ = controller_lowcmd_.tau_ff[7] * whl_side_sign[1];
       control_cmd->cmds_[1].cmd_pack[0].p_cmd_ = 0.0;
       control_cmd->cmds_[1].cmd_pack[0].v_cmd_ = 0.0;
       control_cmd->cmds_[1].cmd_pack[0].kp_ = 0.0;
       control_cmd->cmds_[1].cmd_pack[0].kd_ = 0.0;
       control_cmd->cmds_[1].cmd_pack[0].t_ff_ = controller_lowcmd_.tau_des[6] * whl_side_sign[0];

       control_cmd->cmds_[1].cmd_pack[1].p_cmd_ = 0.0;
       control_cmd->cmds_[1].cmd_pack[1].v_cmd_ = 0.0;
       control_cmd->cmds_[1].cmd_pack[1].kp_ = 0.0;
       control_cmd->cmds_[1].cmd_pack[1].kd_ = 0.0;
       control_cmd->cmds_[1].cmd_pack[1].t_ff_ = controller_lowcmd_.tau_des[7] * whl_side_sign[1];

       for (int i = 9; i < 15; i++)
    {
        // control_cmd->cmds_[i/6].cmd_pack[i-6*(i/6)].p_cmd_ = (controller_lowcmd_.q_des[i-1] / leg_side_sign[i-3]) + leg_offset[i-3];
        // control_cmd->cmds_[i/6].cmd_pack[i-6*(i/6)].v_cmd_ = controller_lowcmd_.qd_des[i-1] / leg_side_sign[i-3];
        // control_cmd->cmds_[i/6].cmd_pack[i-6*(i/6)].kp_ = controller_lowcmd_.kp_joint[i-1];
        // control_cmd->cmds_[i/6].cmd_pack[i-6*(i/6)].kd_ = controller_lowcmd_.kd_joint[i-1];
        // control_cmd->cmds_[i/6].cmd_pack[i-6*(i/6)].t_ff_ = controller_lowcmd_.tau_ff[i-1]* leg_side_sign[i-3];
        control_cmd->cmds_[i/6].cmd_pack[i-6*(i/6)].p_cmd_ = 0.0;
        control_cmd->cmds_[i/6].cmd_pack[i-6*(i/6)].v_cmd_ = 0.0;
        control_cmd->cmds_[i/6].cmd_pack[i-6*(i/6)].kp_ = 0.0;
        control_cmd->cmds_[i/6].cmd_pack[i-6*(i/6)].kd_ = 0.0;
        control_cmd->cmds_[i/6].cmd_pack[i-6*(i/6)].t_ff_ = controller_lowcmd_.tau_des[i-1]* leg_side_sign[i-3];
    }

    //    control_cmd->cmds_[2].cmd_pack[3].p_cmd_ = (controller_lowcmd_.q_des[14] / whl_side_sign[2]) + whl_offset[2];
    //    control_cmd->cmds_[2].cmd_pack[3].v_cmd_ = controller_lowcmd_.qd_des[14] / whl_side_sign[2];
    //    control_cmd->cmds_[2].cmd_pack[3].kp_ = controller_lowcmd_.kp_joint[14];
    //    control_cmd->cmds_[2].cmd_pack[3].kd_ = controller_lowcmd_.kd_joint[14];
    //    control_cmd->cmds_[2].cmd_pack[3].t_ff_ = controller_lowcmd_.tau_ff[14] * whl_side_sign[2];

    //    control_cmd->cmds_[2].cmd_pack[4].p_cmd_ = (controller_lowcmd_.q_des[15] / whl_side_sign[3]) + whl_offset[3];
    //    control_cmd->cmds_[2].cmd_pack[4].v_cmd_ = controller_lowcmd_.qd_des[15] / whl_side_sign[3];
    //    control_cmd->cmds_[2].cmd_pack[4].kp_ = controller_lowcmd_.kp_joint[15];
    //    control_cmd->cmds_[2].cmd_pack[4].kd_ = controller_lowcmd_.kd_joint[15];
    //    control_cmd->cmds_[2].cmd_pack[4].t_ff_ = controller_lowcmd_.tau_ff[15] * whl_side_sign[3];
       control_cmd->cmds_[2].cmd_pack[3].p_cmd_ = 0.0;
       control_cmd->cmds_[2].cmd_pack[3].v_cmd_ = 0.0;
       control_cmd->cmds_[2].cmd_pack[3].kp_ = 0.0;
       control_cmd->cmds_[2].cmd_pack[3].kd_ = 0.0;
       control_cmd->cmds_[2].cmd_pack[3].t_ff_ = controller_lowcmd_.tau_des[14] * whl_side_sign[2];

       control_cmd->cmds_[2].cmd_pack[4].p_cmd_ = 0.0;
       control_cmd->cmds_[2].cmd_pack[4].v_cmd_ = 0.0;
       control_cmd->cmds_[2].cmd_pack[4].kp_ = 0.0;
       control_cmd->cmds_[2].cmd_pack[4].kd_ =0.0;
       control_cmd->cmds_[2].cmd_pack[4].t_ff_ = controller_lowcmd_.tau_des[15] * whl_side_sign[3];

    if (count_usb_send<60)
    {
        control_cmd->cmds_[0].chip_flag[0] = (0x01 << 1);
        control_cmd->cmds_[1].chip_flag[0] = (0x01 << 1);
        control_cmd->cmds_[2].chip_flag[0] = (0x01 << 1);
    }else{
        control_cmd->cmds_[0].chip_flag[0] = (0x01) | (0x01 << 2);
        control_cmd->cmds_[1].chip_flag[0] = (0x01) | (0x01 << 2);
        control_cmd->cmds_[2].chip_flag[0] = (0x01) | (0x01 << 2);
        // std::cout<<"enable"<<std::endl;
    }
    // if (count_usb_send>10)
    // {
    //     first_run_in_lcmsend=true;
    // }
    if (count_usb_send<100)
    {
        count_usb_send++;
    }
    mutex_cmd_data.unlock();
}

 void Beast_USB2CAN::imu_compensation(IMUData* source, IMUData* final)
{
    eigen_gyro(0) = source->gyro[0];
    eigen_gyro(1) = source->gyro[1];
    eigen_gyro(2) = source->gyro[2];

    eigen_acc(0) = source->accel[0];
    eigen_acc(1) = source->accel[1];
    eigen_acc(2) = source->accel[2];

    eigen_quat(0) = source->q[0];
    eigen_quat(1) = source->q[1];
    eigen_quat(2) = source->q[2];
    eigen_quat(3) = source->q[3];

    //rotate by eigen calculation.
    eigen_gyro_rotated = rot_offset*eigen_gyro;
    eigen_acc_rotated = rot_offset*eigen_acc;
    eigen_quat_rotated = rotationMatrixToQuaternion(rot_offset*quaternionToRotationMatrix(eigen_quat));
    // write
    final->gyro[0] = eigen_gyro_rotated(0);
    final->gyro[1] = eigen_gyro_rotated(1);
    final->gyro[2] = eigen_gyro_rotated(2);

    final->accel[0] = eigen_acc_rotated(0);
    final->accel[1] = eigen_acc_rotated(1);
    final->accel[2] = eigen_acc_rotated(2);

    final->q[0] = eigen_quat_rotated(0);
    final->q[1] = eigen_quat_rotated(1);
    final->q[2] = eigen_quat_rotated(2);
    final->q[3] = eigen_quat_rotated(3);
}

void Beast_USB2CAN::_thread_sen_run(){
    if(rc_control.mode==RC_mode::LOWLEVELSDK){
        publish_data2Controller();
    }
}

void  Beast_USB2CAN::thread_run_xbox(){
    while (true) {
        js_complete(port_xbox);
        if (rc_control.mode==RC_mode::LOWLEVELSDK)
        {
            if (Normal_print_count%1000==0)
            {
                 std::cerr << "Normal in LOWLEVELSDK." << std::endl;
            }
            Normal_print_count++;
        }
    }
    
    if (Normal_print_count>20000)Normal_print_count=0;
    
};
