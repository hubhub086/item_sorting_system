#ifndef _UART_H_
#define _UART_H_

int uart_init(const char* tty,  int baud_rate, int data_bits, char parity, int stop_bits);
void uart_receiveThread_touch();
void uart_send(int fd, const char* str);

int open_port(const char*);
int set_uart_config(int fd, int baud_rate, int data_bits, char parity, int stop_bits);
void* receive_thread(void* ptr);

#endif