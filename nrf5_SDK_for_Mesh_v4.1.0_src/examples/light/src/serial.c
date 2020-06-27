#include "serial.h"
#include "serial_uart.h"

#include "serial_fifo.h"
#include "nrf_mesh_assert.h"

#include "log.h"

typedef enum
{
    SERIAL_STATE_IDLE,
    SERIAL_STATE_TRANSMIT
} serial_state_t;

static serial_state_t serial_state = SERIAL_STATE_IDLE;

#define SERIAL_FIFO_BUFFER_SIZE 64

static serial_fifo_t serial_input_fifo;
static serial_fifo_t serial_output_fifo;

static char serial_input_buffer[SERIAL_FIFO_BUFFER_SIZE];
static char serial_output_buffer[SERIAL_FIFO_BUFFER_SIZE];

static serial_rx_cb_t serial_byte_rx_cb;
static void serial_char_rx(uint8_t c)
{
    serial_byte_rx_cb(c);
    serial_fifo_write(&serial_input_fifo, &c, 1);
}

static void serial_char_tx(void)
{
    /* Unexpected event */
    NRF_MESH_ASSERT(serial_state != SERIAL_STATE_IDLE);

    if (serial_fifo_is_empty(&serial_output_fifo))
    {
        /* We have nothing to send. */
        serial_uart_tx_stop();
        serial_state = SERIAL_STATE_IDLE;
    }
    else
    {
        uint8_t value;
        serial_fifo_read(&serial_output_fifo, &value, 1);
        serial_uart_byte_send(value);
    }
}


void serial_init(serial_rx_cb_t byte_rx_cb){

    serial_byte_rx_cb = byte_rx_cb;

    // Init Input and Output FIFO
    serial_fifo_init(&serial_input_fifo, serial_input_buffer, SERIAL_FIFO_BUFFER_SIZE);
    serial_fifo_init(&serial_output_fifo, serial_output_buffer, SERIAL_FIFO_BUFFER_SIZE);

    NRF_MESH_ASSERT(NRF_SUCCESS == serial_uart_init(serial_char_rx, serial_char_tx));
    serial_uart_receive_set(true);
}

void serial_send(unsigned char * buffer, int size){

    __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO, "serial cmd ", buffer, size);

    serial_fifo_write(&serial_output_fifo,buffer,size);
    if(serial_state==SERIAL_STATE_IDLE){
        serial_state=SERIAL_STATE_TRANSMIT;
        serial_uart_tx_start();
        serial_char_tx();
    }
}

int serial_receive(unsigned char * buffer,int size) {
    int bytes_read = serial_fifo_read(&serial_input_fifo,buffer,size);
    return bytes_read;
}

void serial_process() {
    serial_uart_process();
}

void serial_send_on_off_over_uart(uint8_t value){
    // Default is OFF
    unsigned char message[] = "\xF5\xAF\x01\x01\x00";
    unsigned char message_length = 5;

    // Setting value to the message
    if (value) {
        // Change to ON
        message[4] = '\x01';
    }

    serial_send(message, message_length);
}