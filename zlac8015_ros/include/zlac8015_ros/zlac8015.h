#ifndef _ZLAC8015_H_
#define _ZLAC8015_H_
#include <iostream>
#include <chrono>
#include <vector>
#include <string>
#include "zlac8015_ros/serial.h"
#include "zlac8015_ros/crc_check.h"


namespace PGV
{
    class ZLAC
    {
    
    protected:


        uint8_t hex_cmd[8] = {0};
        uint8_t receive_hex[15] = {0};
        uint8_t ID = 0x00;
        const uint8_t READ = 0x03;
        const uint8_t WRITE = 0x06;
        const uint8_t CONTROL_REG[2] = {0X20, 0X31};
        const uint8_t ENABLE[2] = {0x00, 0X08};
        const uint8_t DISABLE[2] = {0x00, 0X07};
        const uint8_t OPERATING_MODE[2] = {0X20, 0X32};
        const uint8_t VEL_MODE[2] = {0x00, 0X03};
        const uint8_t SET_RPM[2] = {0x20, 0X3A};
        const uint8_t GET_RPM[2] = {0x20, 0X2C};
        const uint8_t SET_ACC_TIME[2] = {0x20, 0X37};
        const uint8_t SET_DECC_TIME[2] = {0x20, 0X38};
        const uint8_t SET_KP[2] = {0x20, 0X1D};
        const uint8_t SET_KI[2] = {0x20, 0X1E};
        const uint8_t INITIAL_SPEED[2] = {0X20, 0X08};
        const uint8_t MAX_SPEED[2] = {0X20, 0X0A};
        const uint8_t ACTUAL_POSITION_H[2] = {0X20, 0X2A};
        const uint8_t ACTUAL_POSITION_L[2] = {0X20, 0X2B};
        const uint8_t START_POSITION_L[2] = {0X20, 0X36};
        const uint8_t START_POSITION_H[2] = {0X20, 0XAA};


        void calculate_crc();

        
        public:

        PGV::Serial _serial;

        
        ZLAC();

        void begin(std::string port , uint8_t ID = 0x00);

        uint8_t set_vel_mode();

         uint8_t set_acc_time(uint16_t acc_time_ms);

        uint8_t set_decc_time(uint16_t decc_time_ms);

        uint8_t set_kp(uint16_t proportional_gain);

        uint8_t set_ki(uint16_t integral_gain);
        uint8_t enable();

        uint8_t disable();

        uint8_t set_rpm(int16_t rpm);

        float get_rpm();

        int32_t get_position();

        uint16_t get_error();

         uint8_t read_motor();

         uint8_t initial_speed(uint16_t rpm);

         uint8_t max_speed(uint16_t rpm);

         void sleep(unsigned long milliseconds);

         uint8_t read_hex(uint8_t num_bytes);

         void print_hex_cmd() const;

         float get_torque();

         uint8_t set_pos(uint16_t pos);

};
};
#endif