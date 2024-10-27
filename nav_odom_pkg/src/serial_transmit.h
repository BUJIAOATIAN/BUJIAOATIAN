#ifndef SERIAL_SEND_H
#define SERIAL_SEND_H
#include <serial/serial.h>  
#include <std_msgs/String.h>  

serial::Serial sentry_ser; // 声明串口对象

uint8_t data_len = 15; // 更新数据包长度为15（2字节头 + 3个float数据 + 2字节尾）
std::string cmd_vel_topic;

union Serial_Package
{
    #pragma pack (1) 
    struct
    {
        uint8_t  header[2] = {0x0D, 0x0A};  // 两字节包头
        float  linear_x;
        float  linear_y;
        float  angular_z;
        uint8_t tail[2] = {0x55, 0xBE};    // 两字节包尾
    };
    #pragma pack ()
    uint8_t Send_Buffer[16];
};

#endif

