//
// Created by lingwei on 4/3/24.
//
#include "../include/rt_usb_imu.h"
#include "cstring"
#include "std_cout_colors.h"

N_Communication::USB_COM::USB_COM(int in_length_8b, int out_length_8b, Vendor_id_Hex _vendor_id,
                                  Product_id_Hex _product_id)
        : usb_message_in_length(in_length_8b), usb_message_out_length(out_length_8b),
          Vendor_id(_vendor_id), Product_id(_product_id) {
    usb_message_in_checklength = usb_message_in_length / 4 - 1;
    usb_message_out_checklength = usb_message_out_length / 4 - 1;
    usb_message_32_2_8 = usb_message_out_length / 4;
    usb_message_8_2_32 = usb_message_in_length / 4;
    rx_buff = new uint8_t[usb_message_in_length];
    tx_buff = new uint8_t[usb_message_out_length];
    transfer_tx = libusb_alloc_transfer(0);
    transfer_rx = libusb_alloc_transfer(0);

    libusb_init(&ctx);

    deviceHandle = libusb_open_device_with_vid_pid(ctx, Vendor_id.vendor_id, Product_id.product_id);
    if (libusb_kernel_driver_active(deviceHandle, 0x00)) {
        int success = libusb_detach_kernel_driver(deviceHandle, 0x00);
        if (success != 0) {
            std::cout << RED << "[IMU ERROR]: " << RESET << "Detach Driver Failed!" << std::endl;
            std::abort();
        }
    }
    int claim_interface = libusb_claim_interface(deviceHandle, 0x00);
    if (claim_interface != 0) {
        std::cout << RED << "[IMU ERROR]: " << RESET << "Claim Driver Failed!" << std::endl;
        std::abort();
    }
    std::cout << GREEN << "[IMU]: " << RESET <<  "INITIALIZATION SUCCESS!" << std::endl;
}

N_Communication::USB_COM::~USB_COM() {
    try {
        delete[] tx_buff;
        delete[] rx_buff;
        std::cout << "[Release devices]\n";
        libusb_free_transfer(transfer_tx);
        libusb_free_transfer(transfer_rx);
        libusb_release_interface(deviceHandle, 0);
        libusb_close(deviceHandle);
        libusb_exit(nullptr);
    }
    catch (std::exception &e) {
        std::cout << "[Failed] Cannot destroy constructor!\n";
        std::abort();
    }
}

libusb_context *&N_Communication::USB_COM::get_usb_ctx() {
    return ctx;
}

libusb_device_handle *&N_Communication::USB_COM::get_device_handle() {
    return deviceHandle;
}

N_Communication::USB_COM_IMU::USB_COM_IMU(int in_length_8b, int out_length_8b,
                                          N_Communication::Vendor_id_Hex _vendor_id,
                                          N_Communication::Product_id_Hex _product_id,
                                          unsigned char _endpoint_in, unsigned char _endpoit_out) :
        N_Communication::USB_COM(in_length_8b, out_length_8b, _vendor_id, _product_id),
        endpoint_in(_endpoint_in), endpoint_out(_endpoit_out), imu_LCM("udpm://239.255.76.67:7667?ttl=255") {
    usb_in_data = new usb_imu_rx_cmd_t();
    usb_data_drv = new usb_imu_rx_data_t();
    imu_lcm_data = new imu_lcmt();
}

N_Communication::USB_COM_IMU::~USB_COM_IMU() {
    delete usb_in_data;
    delete usb_data_drv;
    delete imu_lcm_data;
}

//void N_Communication::USB_COM_IMU::USB_Com_Start_Trans_Asy(void (*cbf_wrapper)(struct libusb_transfer *)) {
void N_Communication::USB_COM_IMU::USB_Com_Start_Trans_Asy() {
    libusb_fill_bulk_transfer(transfer_rx, get_device_handle(), endpoint_in, rx_buff, usb_message_in_length,
                              imu_cbf_wrapper, this, 0);
    libusb_submit_transfer(transfer_rx);
    if (transfer_rx->status == 0) {
        std::cout << GREEN << "[IMU GOOD]: " << RESET << "USB IMU Asynchronously Receiving!\n";
    } else {
        std::cout << RED << "[IMU ERROR]: " << RESET << "Can not start transmitting\n";
        std::abort();
    }
}

void N_Communication::USB_COM_IMU::Deal_Out_Data() {

}

void N_Communication::USB_COM_IMU::Deal_In_Data() {
    for (int i = 0; i < usb_message_8_2_32; i++) {
        ((uint32_t *) usb_in_data)[i] =
                (rx_buff[4 * i + 3] << 24) + (rx_buff[4 * i + 2] << 16) + (rx_buff[4 * i + 1] << 8) + rx_buff[4 * i];
    }

    auto *temp_cmd = (uint32_t *) usb_in_data;
    uint32_t t = 0;
    for (size_t i = 0; i < usb_message_in_checklength; i++)
        t = t ^ temp_cmd[i];
    if (usb_in_data->checksum == t) {
        {
            std::lock_guard<std::mutex> lock(this->imu_mtx);
            memcpy(usb_data_drv, usb_in_data, usb_message_in_length - 4);
            memcpy(local_imu_data,usb_data_drv,usb_message_in_length - 4 );
            memcpy(imu_lcm_data, usb_data_drv, sizeof(usb_imu_rx_data_t));
            //timestamp: microseconds
            imu_lcm_data->timestamp = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
        }
    } else {
        std::cout << "[ERROR] USB RX CMD CHECKSUM ERROR!\n";
    }
    imu_LCM.publish("IMU_CHANNEL", imu_lcm_data);
}

void N_Communication::USB_COM_IMU::USB_In_CBF(struct libusb_transfer *transfer) {
    // std::cout << "Enter Real CBF!\n";
    if (transfer->status != LIBUSB_TRANSFER_COMPLETED) {
        std::cout << "[ERROR] Asy Trans Failed! Try again!\n";
        libusb_submit_transfer(transfer);
    } else if (transfer->status == LIBUSB_TRANSFER_COMPLETED) {
        //lock_mutex_rx();
        this->Deal_In_Data();
        //unlock_mutex_rx();
        if (if_print)
            this->Print_Rx_Data();
        libusb_submit_transfer(transfer);
    }
}

void N_Communication::USB_COM_IMU::USB_Out_CBF(struct libusb_transfer *transfer) {

}

void N_Communication::USB_COM_IMU::Print_Rx_Data() {
    std::cout.precision(3);
    std::cout << "[IMU " << get_product_id() << "DATA] [" << usb_data_drv->q[0] << " " << usb_data_drv->q[1] << " " <<
              usb_data_drv->q[2] << " " << usb_data_drv->q[3] << "] [" << usb_data_drv->gyro[0] << " " <<
              usb_data_drv->gyro[1] << " " << usb_data_drv->gyro[2] << "] [" << usb_data_drv->accel[0] << " " <<
              usb_data_drv->accel[1] << " " << usb_data_drv->accel[2] << "]" << std::endl;
}

void N_Communication::USB_COM_IMU::usb_imu_set_rx_buffer(USB_Imu_t *imu_data_) {
    local_imu_data = imu_data_;
}

void imu_cbf_wrapper(struct libusb_transfer *_transfer) {
    //std::cout << "Enter wrapper function!!!!!!!!\n";
    auto *temp = reinterpret_cast<N_Communication::USB_COM_IMU *>(_transfer->user_data);
    temp->USB_In_CBF(_transfer);
}





