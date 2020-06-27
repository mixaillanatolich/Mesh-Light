#include "serial_commands.h"
#include "serial.h"
#include "serial_fifo.h"
#include "nrf.h"
#include "timer.h"
#include "app_error.h" 
#include "nrf_mesh_utils.h"

#include "log.h"

#define FIFO_BUFFER_ENABLED 0

// TIMER PREFERENCES

static timestamp_t last_command_timestamp;

#define SERIAL_SEND_TIMEOUT_DELAY_MS  (200)
#define SERIAL_SEND_RETRY_COUNT_MAX   (3)

#define MS_TO_US(t) ((t) * 1000)

// OUTBOUND COMMANDS QUEUE
#define     SERIAL_COMMANDS_QUEUE_SIZE          32
#define     SERIAL_COMMANDS_QUEUE_BUFFER_SIZE   SERIAL_COMMANDS_QUEUE_SIZE*sizeof(serial_command_t)

#if FIFO_BUFFER_ENABLED
static serial_fifo_t serial_commands_queue;
#endif

static char serial_commands_queue_buffer[SERIAL_COMMANDS_QUEUE_BUFFER_SIZE];
static serial_commands_queue_state_t serial_commands_queue_state;

static serial_command_t last_command_sent;

#if !FIFO_BUFFER_ENABLED
static serial_command_t current_cntrl_command;
static serial_command_t current_ack_command;
#endif

static uint8_t  retry_count;

// Registered command receive callback
static serial_command_rx_cb_t   command_rx_callback;

// Registered status callback
static serial_status_cb_t   status_callback;

// Command being currently received
static serial_command_t         comm_rx;
// Index of data byte being received within current command
static uint8_t                      comm_rx_data_index;
// Command RX state machine state
static serial_command_rx_state_t comm_rx_state;

void serial_commands_init(serial_command_rx_cb_t command_rx_cb, serial_status_cb_t status_cb) {
    // Init outbound commands queue (buffer  + state machine)
 #if FIFO_BUFFER_ENABLED
    serial_fifo_init(&serial_commands_queue, serial_commands_queue_buffer, SERIAL_COMMANDS_QUEUE_BUFFER_SIZE);
 #endif
    serial_commands_queue_state = COMMAND_QUEUE_IDLE;

    // Set command receive callback
    command_rx_callback = command_rx_cb;

    status_callback = status_cb;

    // Init command RX state machine
    comm_rx_state = COMM_RX_START_BYTE_1;

    // Init Serial hardware
    serial_init(serial_command_byte_received);
}



void serial_command_send_to_serial(serial_command_t command) {
    uint8_t     data_to_send[COMMAND_MAX_LENGTH];
    uint16_t    data_to_send_length = command.length + EMPTY_COMMAND_LENGTH;
    uint8_t *   payload = &data_to_send[COMMAND_START_LENGTH];
    uint16_t    payload_length = command.length+EMPTY_COMMAND_PAYLOAD_LENGTH;

    // Setting Start Sequence
    data_to_send[0] = 0xF5;
    data_to_send[1] = 0xAF;

    // Forming the payload
    payload[0] = command.length;
    payload[1] = command.command_id;

    for (int i=0; i<command.length; i++){
        payload[EMPTY_COMMAND_PAYLOAD_LENGTH+i] = command.data[i];
    }

    // Sending the command out
    serial_send(data_to_send, data_to_send_length);
}

int serial_command_queue_is_empty(){
#if FIFO_BUFFER_ENABLED
    return serial_fifo_is_empty(&serial_commands_queue);
#else 
    return current_cntrl_command.length == 0 && current_ack_command.length == 0;
#endif
}

#if FIFO_BUFFER_ENABLED
void serial_command_queue_enqueue_command(serial_command_t command) {
    serial_fifo_write(&serial_commands_queue,&command,sizeof(command));
}
#endif

serial_command_t serial_command_queue_dequeue_command(){
    serial_command_t command;

#if FIFO_BUFFER_ENABLED
    serial_fifo_read(&serial_commands_queue,&command,sizeof(command));
#else 
    if (current_ack_command.length != 0) {
        command = current_ack_command;
        current_ack_command.length = 0;
    } else if (current_cntrl_command.length != 0) {
        command = current_cntrl_command;
        current_cntrl_command.length = 0;
    }
#endif

    return command;
}



void serial_send_next_command(){
    retry_count = 0;

    if (!serial_command_queue_is_empty()) {
        // Setting state machine state  in WAITING_ACK
        last_command_timestamp = timer_now();
        serial_commands_queue_state = COMMAND_QUEUE_WAITING_ACK;
        last_command_sent = serial_command_queue_dequeue_command();
        serial_command_send_to_serial(last_command_sent);
    } else {
    //    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "SERIAL: No more commands found\n");
        serial_commands_queue_state = COMMAND_QUEUE_IDLE;
    }
}

void serial_retry_send_command(){
    if (retry_count<SERIAL_SEND_RETRY_COUNT_MAX) {
        retry_count++;
      //  __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "SERIAL: Retrying send command. Attempt %d\n",retry_count);
        last_command_timestamp = timer_now();
        serial_commands_queue_state = COMMAND_QUEUE_WAITING_ACK;
        serial_command_send_to_serial(last_command_sent);
    } else {
    //    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "SERIAL: Retry attempt count exceeded. Sending the next command\n");
        status_callback(false);
        serial_send_next_command();
    }
}

void serial_commands_process(){
    if (serial_commands_queue_state == COMMAND_QUEUE_WAITING_ACK) {
        if(timer_now()>last_command_timestamp+MS_TO_US(SERIAL_SEND_TIMEOUT_DELAY_MS)){
         //   __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "SERIAL: No ACK received. Try to send the command again\n");
            serial_retry_send_command();
        }
    }
}

// Generic command send routine
void serial_send_command(serial_command_t command) {
#if FIFO_BUFFER_ENABLED
    serial_command_queue_enqueue_command(command);
#endif
    switch (serial_commands_queue_state) {
        case COMMAND_QUEUE_IDLE:
            serial_send_next_command();
            break;
        case COMMAND_QUEUE_WAITING_ACK:
            break;
    }
}

// Specific commands send
void serial_send_on_off_command(uint8_t state) {
    uint8_t command_code = 0x01;

    serial_command_t command;
    command.length = 1;
    command.command_id = command_code;
    command.data[0] = state;

#if !FIFO_BUFFER_ENABLED
    current_cntrl_command = command;
#endif

    serial_send_command(command);
}

void serial_send_ack_command(uint8_t ack) {
    uint8_t command_code = 0xF0;

    serial_command_t command;
    command.length = 1;
    command.command_id = command_code;
    command.data[0] = ack;

#if !FIFO_BUFFER_ENABLED
    current_ack_command = command;
#endif
    
    serial_send_command(command);
}


// Verify the command and send out the external callback if the command is valid
static void serial_command_received(serial_command_t command){
    // Verify CRC16
//     uint8_t payload[COMMAND_MAX_LENGTH];
// 
//     uint16_t payload_length = EMPTY_COMMAND_PAYLOAD_LENGTH + command.length;
// 
//     payload[0] = command.length;
//     payload[1] = command.command[0];
//     payload[2] = command.command[1];
//     for (int i=0; i<command.length; i++){
//         payload[EMPTY_COMMAND_PAYLOAD_LENGTH+i] = command.data[i];
//     }
// 
//     uint16_t crc = crc16(payload, payload_length);
// 
//     if ((crc&0xff)==command.crc16[0] && (crc>>8)==command.crc16[1]){
//         //CRC is valid
//         // Check the command code
        uint8_t command_code = command.command_id;
        int command_is_valid = 1;
        switch(command_code) {
            case SERIAL_COMMAND_ACK:
                // ACK/NACK
                if (command.data[0]==0){
                    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "SERIAL: ACK received. Sending the next command\n");
                    serial_send_next_command();
                    status_callback(true);
                } else {
                    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "SERIAL: NACK received. Trying to send the command again\n");
                    serial_retry_send_command();
                }
                break;
            default:
                // INVALID COMMAND
                command_is_valid= 0;
        }

        if (command_is_valid) {
            command_rx_callback (command);
        } else {
            // Invalid command code
            // NACK should be sent here
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "SERIAL: Invalid Command\n");
        }

//     } else {
//         // Invalid CRC, command is not accepted
//         // NACK should be sent here
//         __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "SERIAL: Invalid CRC16\n");
// 
//     }
}

void serial_command_byte_received(uint8_t byte) {

    switch (comm_rx_state) {
        case COMM_RX_START_BYTE_1:
            if (byte==0xF5){
                comm_rx_state = COMM_RX_START_BYTE_2;
            }
            break;
        case COMM_RX_START_BYTE_2:
            if (byte==0xAF){
                comm_rx_state = COMM_RX_LENGTH;
            } else {
                comm_rx_state = COMM_RX_START_BYTE_1;
            }
            break;
        case COMM_RX_LENGTH:
            if (byte>=0 && byte<=64){
                comm_rx.length = byte;
                comm_rx_state = COMM_RX_COMMAND_BYTE;
            } else {
                comm_rx_state = COMM_RX_START_BYTE_1;
            }
            break;
        case COMM_RX_COMMAND_BYTE:
            comm_rx.command_id = byte;
            comm_rx_data_index = 0;
            if (comm_rx.length > 0){
                comm_rx_state = COMM_RX_DATA;
            } else {
                comm_rx_state = COMM_RX_START_BYTE_1;
                serial_command_received(comm_rx);
            }
            break;
        case COMM_RX_DATA:
            comm_rx.data[comm_rx_data_index] = byte;
            comm_rx_data_index++;
            if (comm_rx_data_index==comm_rx.length){
                comm_rx_state = COMM_RX_START_BYTE_1;
            	serial_command_received(comm_rx);
            }
            break;
    }
}

