#include "zlac8015_ros/zlac8015.h"


PGV::ZLAC::ZLAC()
{


}

void PGV::ZLAC::begin(std::string port, uint8_t ID)
{
    this->ID = ID;
    _serial.begin(port);

}



uint8_t PGV::ZLAC::set_vel_mode()
{
   hex_cmd[0] = ID;
    hex_cmd[1] = WRITE;
    hex_cmd[2] = OPERATING_MODE[0];
    hex_cmd[3] = OPERATING_MODE[1];

    hex_cmd[4] = VEL_MODE[0];
    hex_cmd[5] = VEL_MODE[1];

    calculate_crc();

    _serial.write(std::vector<uint8_t>(std::begin(hex_cmd),std::end(hex_cmd)));
    //read_hex(8);
    return 0;
}


uint8_t PGV::ZLAC::set_acc_time(uint16_t acc_time_ms)
{
    // memset(hex_cmd, 0, sizeof(hex_cmd));
    hex_cmd[0] = ID;
    hex_cmd[1] = WRITE;
    hex_cmd[2] = SET_ACC_TIME[0];
    hex_cmd[3] = SET_ACC_TIME[1];

    hex_cmd[4] = (acc_time_ms >> 8) & 0xFF;
    hex_cmd[5] = acc_time_ms & 0xFF;

    calculate_crc();
    _serial.write(std::vector<uint8_t>(std::begin(hex_cmd),std::end(hex_cmd)));
    if (read_hex(8))
        return 1;
    return 0;
}

uint8_t PGV::ZLAC::set_decc_time(uint16_t decc_time_ms)
{
    // memset(hex_cmd, 0, sizeof(hex_cmd));
    hex_cmd[0] = ID;
    hex_cmd[1] = WRITE;
    hex_cmd[2] = SET_DECC_TIME[0];
    hex_cmd[3] = SET_DECC_TIME[1];

    hex_cmd[4] = (decc_time_ms >> 8) & 0xFF;
    hex_cmd[5] = decc_time_ms & 0xFF;

    calculate_crc();
    _serial.write(std::vector<uint8_t>(std::begin(hex_cmd),std::end(hex_cmd)));
    if (read_hex(8))
        return 1;
    return 0;
}


uint8_t PGV::ZLAC::set_kp(uint16_t proportional_gain)
{
    // memset(hex_cmd, 0, sizeof(hex_cmd));
    hex_cmd[0] = ID;
    hex_cmd[1] = WRITE;
    hex_cmd[2] = SET_KP[0];
    hex_cmd[3] = SET_KP[1];

    hex_cmd[4] = (proportional_gain >> 8) & 0xFF;
    hex_cmd[5] = proportional_gain & 0xFF;

    calculate_crc();
    _serial.write(std::vector<uint8_t>(std::begin(hex_cmd),std::end(hex_cmd)));
    if (read_hex(8))
        return 1;
    return 0;
}


uint8_t PGV::ZLAC::set_ki(uint16_t integral_gain)
{
    // memset(hex_cmd, 0, sizeof(hex_cmd));
    hex_cmd[0] = ID;
    hex_cmd[1] = WRITE;
    hex_cmd[2] = SET_KI[0];
    hex_cmd[3] = SET_KI[1];

    hex_cmd[4] = (integral_gain >> 8) & 0xFF;
    hex_cmd[5] = integral_gain & 0xFF;

    calculate_crc();
    _serial.write(std::vector<uint8_t>(std::begin(hex_cmd),std::end(hex_cmd)));
    if (read_hex(8))
        return 1;
    return 0;
}


uint8_t PGV::ZLAC::enable()
{
    memset(hex_cmd, 0, sizeof(hex_cmd));
    hex_cmd[0] = ID;
    hex_cmd[1] = WRITE;
    hex_cmd[2] = CONTROL_REG[0];
    hex_cmd[3] = CONTROL_REG[1];

    hex_cmd[4] = ENABLE[0];
    hex_cmd[5] = ENABLE[1];

    calculate_crc();
    _serial.write(std::vector<uint8_t>(std::begin(hex_cmd),std::end(hex_cmd)));
    if (read_hex(8))
        return 1;
    return 0;
}


uint8_t PGV::ZLAC::disable()
{
    // memset(hex_cmd, 0, sizeof(hex_cmd));
    hex_cmd[0] = ID;
    hex_cmd[1] = WRITE;
    hex_cmd[2] = CONTROL_REG[0];
    hex_cmd[3] = CONTROL_REG[1];

    hex_cmd[4] = DISABLE[0];
    hex_cmd[5] = DISABLE[1];

    calculate_crc();
    _serial.write(std::vector<uint8_t>(std::begin(hex_cmd),std::end(hex_cmd)));
    if (read_hex(8))
        return 1;
    return 0;
}

void PGV::ZLAC::calculate_crc()
{
    // calculate crc and append to hex cmd
    unsigned short result = crc16(hex_cmd, sizeof(hex_cmd) - 2);
    hex_cmd[6] = result & 0xFF;
    hex_cmd[7] = (result >> 8) & 0xFF;
}


uint8_t PGV::ZLAC::set_rpm(int16_t rpm)
{
   
    hex_cmd[0] = ID;
    hex_cmd[1] = WRITE;
    hex_cmd[2] = SET_RPM[0];
    hex_cmd[3] = SET_RPM[1];

    hex_cmd[4] = (rpm >> 8) & 0xFF;
    hex_cmd[5] = rpm & 0xFF;

    calculate_crc();
    _serial.write(std::vector<uint8_t>(std::begin(hex_cmd),std::end(hex_cmd)));
    read_hex(8);
    return 0;
}


float PGV::ZLAC::get_rpm()
{
 
    int16_t rpm_tenth = receive_hex[8] + (receive_hex[7] << 8);
    return (float)rpm_tenth / 10.0f;
}


int32_t PGV::ZLAC::get_position()
{

    return receive_hex[6] + (receive_hex[5] << 8) + (receive_hex[4] << 16) + (receive_hex[3] << 24);
}

uint16_t PGV::ZLAC::get_error()
{
    return receive_hex[12] + (receive_hex[11] << 8);
}




uint8_t PGV::ZLAC::read_motor()
{
    // memset(hex_cmd, 0, sizeof(hex_cmd));
    hex_cmd[0] = ID;
    hex_cmd[1] = READ;
    hex_cmd[2] = ACTUAL_POSITION_H[0];
    hex_cmd[3] = ACTUAL_POSITION_H[1];
    hex_cmd[4] = 0x00;
    hex_cmd[5] = 0x05;

    calculate_crc();

    _serial.write(std::vector<uint8_t>(std::begin(hex_cmd),std::end(hex_cmd)));

    
    
    
    std::vector<uint8_t> data;
    _serial.read(data,15);
    for(int i = 0 ; i < data.size() ; i++)
    {
        receive_hex[i] = data[i];

    }
    
    if (crc16(receive_hex, 15) != 0)
    {
        _serial.flush();
      
        return 1;
    }
    

    return 0;
}


uint8_t PGV::ZLAC::initial_speed(uint16_t rpm)
{
    // memset(hex_cmd, 0, sizeof(hex_cmd));
    hex_cmd[0] = ID;
    hex_cmd[1] = WRITE;
    hex_cmd[2] = INITIAL_SPEED[0];
    hex_cmd[3] = INITIAL_SPEED[1];

    hex_cmd[4] = (rpm >> 8) & 0xFF;
    hex_cmd[5] = rpm & 0xFF;

    calculate_crc();
    _serial.write(std::vector<uint8_t>(std::begin(hex_cmd),std::end(hex_cmd)));
    if (read_hex(8))
        return 1;
    return 0;
}



uint8_t PGV::ZLAC::max_speed(uint16_t rpm)
{
    // memset(hex_cmd, 0, sizeof(hex_cmd));
    hex_cmd[0] = ID;
    hex_cmd[1] = WRITE;
    hex_cmd[2] = MAX_SPEED[0];
    hex_cmd[3] = MAX_SPEED[1];

    hex_cmd[4] = (rpm >> 8) & 0xFF;
    hex_cmd[5] = rpm & 0xFF;

    calculate_crc();
    _serial.write(std::vector<uint8_t>(std::begin(hex_cmd),std::end(hex_cmd)));
    if (read_hex(8))
        return 1;
    return 0;
}


void PGV::ZLAC::sleep(unsigned long milliseconds)
{
#ifdef _WIN32
    Sleep(milliseconds); // 100 ms
#else
{
    usleep(milliseconds * 1000); // 100 ms
#endif
}
}

uint8_t PGV::ZLAC::read_hex(uint8_t num_bytes)
{
    
    
     std::vector<uint8_t> data;
    _serial.read(data,num_bytes);
    for(int i = 0 ; i < data.size() ; i++)
    {
        receive_hex[i] = data[i];

    }
    
    // crc check of received data
    if (crc16(receive_hex, num_bytes) != 0)
    {
        
        return 1;
    }
    return 0;
}


float PGV::ZLAC::get_torque()
{
    int16_t torque = receive_hex[10] + (receive_hex[9] << 8);
    return (float)torque / 10.0f;
}



void PGV::ZLAC::print_hex_cmd() const
{
    
    for (int i = 0; i < 8; i++)
    {
        printf("%d, %02x\n", i, hex_cmd[i]);
    }
}


uint8_t PGV::ZLAC::set_pos(uint16_t pos)
{
    // memset(hex_cmd, 0, sizeof(hex_cmd));
    hex_cmd[0] = ID;
    hex_cmd[1] = WRITE;
    hex_cmd[2] = START_POSITION_L[0];
    hex_cmd[3] = START_POSITION_L[1];
    hex_cmd[4] = 0xFF;
    hex_cmd[5] = 0xFF;

    calculate_crc();

    _serial.write(std::vector<uint8_t>(std::begin(hex_cmd),std::end(hex_cmd)));



    hex_cmd[0] = ID;
    hex_cmd[1] = WRITE;
    hex_cmd[2] = START_POSITION_H[0];
    hex_cmd[3] = START_POSITION_H[1];
    hex_cmd[4] = 0xFF;
    hex_cmd[5] =  0xFF;

    calculate_crc();

    _serial.write(std::vector<uint8_t>(std::begin(hex_cmd),std::end(hex_cmd)));
}