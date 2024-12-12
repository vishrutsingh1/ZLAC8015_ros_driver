#include "zlac8015_ros/serial.h"
#include <iostream>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <cstring>

PGV::Serial::Serial() : _bufferSize(1024)
{

   
}


void PGV::Serial::begin(const std::string &device, const Baud baud)
{
     _fd = ::open(device.c_str(), O_RDWR);

    if (_fd < 0)
    {
        std::cout << "Serial: can't open device " << device << "." << std::endl;
        return;
    }

    struct termios tty;

    if (tcgetattr(_fd, &tty))
    {
        std::cout << "Serial: can't get the attributes from the device " << device << "." << std::endl;
        return;
    }

    cfsetospeed(&tty, baud);
    cfsetispeed(&tty, baud);
    tty.c_cflag |= PARENB;
    tty.c_cflag &= ~PARODD;

    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ECHONL;
    tty.c_lflag &= ~ISIG;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;
    tty.c_cc[VTIME] = 5;
    tty.c_cc[VMIN] = 0;

    if (tcsetattr(_fd, TCSANOW, &tty))
    {
        std::cout << "Rs485: can't set the attributes to the device " << device << "." << std::endl;
        return;
    }
}

PGV::Serial::~Serial(void)
{
    ::close(_fd);
}

void PGV::Serial::write(const std::vector<uint8_t> &bytes)
{
    ::write(_fd, bytes.data(), bytes.size());
}

bool PGV::Serial::read(std::vector<unsigned char> &data, const unsigned int bytes, const int timeout)
{
    data.resize(bytes);

    int n = ::read(_fd, data.data(), data.size());

    return n == data.size();
}

void PGV::Serial::flush(void) {
            if (tcflush(_fd, TCIOFLUSH) != 0) {
                std::cerr << "Serial: Error flushing the serial port." << std::endl;
            }
        }
