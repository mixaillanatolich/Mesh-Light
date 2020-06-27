#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdint.h>

typedef void (*serial_rx_cb_t)(uint8_t c);

void serial_init(serial_rx_cb_t byte_rx_cb);
void serial_send(unsigned char * buffer, int size);
void serial_process();
void send_level_over_uart(uint16_t value);
void send_on_off_over_uart(uint8_t value);
int serial_receive(unsigned char * buffer,int size);

#endif /*__SERIAL_H*/