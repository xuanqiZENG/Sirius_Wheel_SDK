//
// Created by lingwei on 3/25/24.
//

#ifndef SIRIUS_SOFT_RT_IMU_H
#define SIRIUS_SOFT_RT_IMU_H

#include "sys/select.h"
#include "libusb-1.0/libusb.h"
#include <ctime>
#include <iostream>
#include <thread>
#include <mutex>
#include <exception>
#include "lcm/lcm-cpp.hpp"
#include "hardware_types.h"
#include "../../lcm-types/cpp/imu_lcmt.hpp"

namespace N_Communication{

    struct Vendor_id_Hex{
        explicit Vendor_id_Hex(uint16_t _vendor_id) : vendor_id(_vendor_id){}
        uint16_t vendor_id;
    };

    struct Product_id_Hex{
        explicit Product_id_Hex(uint16_t _product_id_Hex) : product_id(_product_id_Hex){}
        uint16_t product_id;
    };

    class USB_COM{
    public:
        explicit USB_COM(int in_length_8b, int out_length_8b ,Vendor_id_Hex _vendor_id, Product_id_Hex _product_id);
        virtual ~USB_COM();
        virtual void Deal_Out_Data() = 0;
        virtual void Deal_In_Data() = 0;
        //virtual void USB_Com_Start_Trans_Asy(void(*cbf_wrapper)(struct libusb_transfer*)) = 0;
        virtual void USB_In_CBF(struct libusb_transfer* transfer)= 0;
        virtual void USB_Out_CBF(struct libusb_transfer* transfer)=0;
        libusb_context*& get_usb_ctx();
        libusb_device_handle*& get_device_handle();
        [[nodiscard]] uint16_t get_product_id() const{return Product_id.product_id;}
        //maybe private+function is much better
    protected:
        uint8_t* tx_buff;
        uint8_t* rx_buff;
        int usb_message_in_length;
        int usb_message_out_length;
        int usb_message_in_checklength;
        int usb_message_out_checklength;
        int usb_message_32_2_8;
        int usb_message_8_2_32;

        libusb_transfer* transfer_tx;
        libusb_transfer* transfer_rx;
        libusb_context* ctx;
    private:
        Vendor_id_Hex Vendor_id;
        Product_id_Hex Product_id;
        libusb_device_handle* deviceHandle;
    };
}

namespace N_Communication {
    typedef struct {
        float q[4];
        float gyro[3];
        float accel[3];
        uint32_t checksum;
    } usb_imu_rx_cmd_t;

    typedef struct {
        float q[4];
        float gyro[3];
        float accel[3];
    } usb_imu_rx_data_t;

    class USB_COM_IMU : public N_Communication::USB_COM {
    public:
        USB_COM_IMU(int in_length_8b, int out_length_8b, N_Communication::Vendor_id_Hex _vendor_id,
                    N_Communication::Product_id_Hex _product_id, unsigned char _endpoint_in,
                    unsigned char _endpoit_out);

        ~USB_COM_IMU() override;

        void Deal_Out_Data() override;

        void Deal_In_Data() override;

        //void USB_Com_Start_Trans_Asy(void(*cbf_wrapper)(struct libusb_transfer *)) override;
        void USB_Com_Start_Trans_Asy();

        void USB_In_CBF(struct libusb_transfer *transfer) override;

        void USB_Out_CBF(struct libusb_transfer *transfer) override;

        void Print_Rx_Data();

        inline const usb_imu_rx_data_t *get_usb_received_data() {
            return usb_data_drv;
        }

        void usb_imu_set_rx_buffer(USB_Imu_t * imu_data_);
        std::mutex imu_mtx;
        lcm::LCM imu_LCM;
        imu_lcmt* imu_lcm_data;

    private:
        usb_imu_rx_cmd_t *usb_in_data;
        USB_Imu_t * local_imu_data;
        usb_imu_rx_data_t *usb_data_drv;
        unsigned char endpoint_in;
        unsigned char endpoint_out;
        const int if_print = 0;
    };
}

void imu_cbf_wrapper(struct libusb_transfer* _transfer);

#endif //SIRIUS_SOFT_RT_IMU_H
