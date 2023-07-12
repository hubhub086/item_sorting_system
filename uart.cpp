#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cstdlib>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <iconv.h>
#include <string>
#include <iostream>
#include <pthread.h>

#include "uart.h"

//全局变量
char receive_buffer[128];
int UART_fd;
extern int main_status;
/*
* 打开串口
*/
int open_port(const char* com_port)
{
    int fd;
    // system(strcat((char *)"echo 123456 | sudo -S sudo chmod 777 ", dev[com_port]));
    //O_NDELAY 同 O_NONBLOCK。
    fd = open(com_port, O_RDWR | O_NOCTTY);
    if (fd < 0)
    {
        perror("open serial port");
        return(-1);
    }
 
    //恢复串口为阻塞状态 
    //非阻塞：fcntl(fd,F_SETFL,FNDELAY)  
    //阻塞：fcntl(fd,F_SETFL,0) 
    if (fcntl(fd, F_SETFL, 0) < 0)
    {
        perror("fcntl F_SETFL\n");
    }
    /*测试是否为终端设备*/
    if (isatty(STDIN_FILENO) == 0)
    {
        perror("standard input is not a terminal device");
    }
 
    return fd;
}
 
/*
* 串口设置
*/
int set_uart_config(int fd, int baud_rate, int data_bits, char parity, int stop_bits)
{
    struct termios opt;
    int speed;
    if (tcgetattr(fd, &opt) != 0)
    {
        perror("tcgetattr");
        return -1;
    }
 
    /*设置波特率*/
    switch (baud_rate)
    {
    case 2400:  speed = B2400;  break;
    case 4800:  speed = B4800;  break;
    case 9600:  speed = B9600;  break;
    case 19200: speed = B19200; break;
    case 38400: speed = B38400; break;
    default:    speed = B115200; break;
    }
    cfsetispeed(&opt, speed);
    cfsetospeed(&opt, speed);
    tcsetattr(fd, TCSANOW, &opt);
 
    opt.c_cflag &= ~CSIZE;
 
    /*设置数据位*/
    switch (data_bits)
    {
    case 7: {opt.c_cflag |= CS7; }break;//7个数据位  
    default: {opt.c_cflag |= CS8; }break;//8个数据位 
    }
 
    /*设置奇偶校验位*/
    switch (parity) //N
    {
    case 'n':case 'N':
    {
        opt.c_cflag &= ~PARENB;//校验位使能     
        opt.c_iflag &= ~INPCK; //奇偶校验使能  
    }break;
    case 'o':case 'O':
    {
        opt.c_cflag |= (PARODD | PARENB);//PARODD使用奇校验而不使用偶校验 
        opt.c_iflag |= INPCK;
    }break;
    case 'e':case 'E':
    {
        opt.c_cflag |= PARENB;
        opt.c_cflag &= ~PARODD;
        opt.c_iflag |= INPCK;
    }break;
    case 's':case 'S': /*as no parity*/
    {
        opt.c_cflag &= ~PARENB;
        opt.c_cflag &= ~CSTOPB;
    }break;
    default:
    {
        opt.c_cflag &= ~PARENB;//校验位使能     
        opt.c_iflag &= ~INPCK; //奇偶校验使能          	
    }break;
    }
 
    /*设置停止位*/
    switch (stop_bits)
    {
    case 1: {opt.c_cflag &= ~CSTOPB; } break;
    case 2: {opt.c_cflag |= CSTOPB; }   break;
    default: {opt.c_cflag &= ~CSTOPB; } break;
    }
 
    /*处理未接收字符*/
    tcflush(fd, TCIFLUSH);
 
    /*设置等待时间和最小接收字符*/
    opt.c_cc[VTIME] = (unsigned char)1000;
    opt.c_cc[VMIN] = 0;
 
    /*关闭串口回显*/
    opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL | NOFLSH);
 
    /*禁止将输入中的回车翻译为新行 (除非设置了 IGNCR)*/
    opt.c_iflag &= ~ICRNL;
    /*禁止将所有接收的字符裁减为7比特*/
    opt.c_iflag &= ~ISTRIP;
 
    /*激活新配置*/
    if ((tcsetattr(fd, TCSANOW, &opt)) != 0)
    {
        perror("tcsetattr");
        return -1;
    }
 
    return 0;
}

int uart_init(const char* tty,  int baud_rate, int data_bits, char parity, int stop_bits)
{
    UART_fd = open_port(tty);
    if (set_uart_config(UART_fd, baud_rate,data_bits,parity, stop_bits) < 0)
	{
		perror("set_com_config");
		exit(1);
	}
    return UART_fd;
}

void uart_send(int fd, const char* str)
{
    write(fd, str, strlen(str));
}

void uart_receiveThread_touch()
{
    pthread_t id;
    int ret = pthread_create(&id, NULL, receive_thread, NULL);
    pthread_detach(id);
}

void* receive_thread(void* ptr)
{
	int datanum = 0;
    while (1)
    {
        datanum =  read(UART_fd,  receive_buffer, sizeof(receive_buffer));
        if(datanum) {
            printf("content:%s\n",receive_buffer);
            if (strcmp(receive_buffer, "start"))
            {
                main_status = 20;
                std::cout << "start" << std::endl;
            }
            // memset(receive_buffer, 0, sizeof(receive_buffer));
        }
    }
    return 0;
}
