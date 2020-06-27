#ifndef __SERIAL_FIFO_H
#define __SERIAL_FIFO_H

typedef struct {
     char * buf;
     int head;
     int tail;
     int size;
} serial_fifo_t;

void serial_fifo_init(serial_fifo_t * f, char * buf, int size);
int serial_fifo_read(serial_fifo_t * f, void * buf, int nbytes);
int serial_fifo_write(serial_fifo_t * f, const void * buf, int nbytes);
int serial_fifo_is_empty(serial_fifo_t * f);

#endif /*__SERIAL_FIFO_H*/