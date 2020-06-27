#ifndef SERIAL_COMMANDS_H__
#define SERIAL_COMMANDS_H__

#include <stdint.h>
#include "stdbool.h"

// Command Data Type
typedef struct {
    uint8_t length;
    uint8_t command_id;
    uint8_t data[64];
} serial_command_t;

// OUTBOUND COMMANDS QUEUE STATE
typedef enum {
    COMMAND_QUEUE_IDLE,
    COMMAND_QUEUE_WAITING_ACK
} serial_commands_queue_state_t;

// COMMAND RX STATE MACHINE STATES
typedef enum
{
    COMM_RX_START_BYTE_1,
    COMM_RX_START_BYTE_2,
    COMM_RX_LENGTH,
    COMM_RX_COMMAND_BYTE,
    COMM_RX_DATA
} serial_command_rx_state_t;

// COMMAND CODES
#define SERIAL_COMMAND_ACK             0xF0
#define SERIAL_COMMAND_LIGHT_ON_OFF    0x01
#define SERIAL_COMMAND_FACTORY_RESET   0xAF

// COMMAND FIELDS LENGTH
#define COMMAND_START_LENGTH       	2
#define COMMAND_LENGTH_LENGTH       1
#define COMMAND_COMM_CODE_LENGTH    1
#define COMMAND_DATA_MAX_LENGTH     64

#define EMPTY_COMMAND_LENGTH        COMMAND_START_LENGTH+\
                                        COMMAND_LENGTH_LENGTH+\
                                        COMMAND_COMM_CODE_LENGTH

// Payload is a command without start sequence
#define EMPTY_COMMAND_PAYLOAD_LENGTH    COMMAND_LENGTH_LENGTH+\
                                            COMMAND_COMM_CODE_LENGTH

#define COMMAND_MAX_LENGTH          EMPTY_COMMAND_LENGTH+\
                                        COMMAND_DATA_MAX_LENGTH


// Callback for serial commands received event
typedef void (*serial_command_rx_cb_t)(serial_command_t command);

// Callback for serial status
typedef void (*serial_status_cb_t)(bool connected);

void serial_commands_init(serial_command_rx_cb_t command_rx_cb, serial_status_cb_t status_cb);
void serial_commands_process();
void serial_send_command(serial_command_t command);
void serial_send_on_off_command(uint8_t state);
void serial_send_ack_command(uint8_t ack);
void serial_command_byte_received(uint8_t byte);

#endif /*SERIAL_COMMANDS_H__*/