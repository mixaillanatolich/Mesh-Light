#define BT_RX 7
#define BT_TX 8

#include <SoftwareSerial.h>
SoftwareSerial btSerial(BT_TX, BT_RX); // RX, TX

typedef enum
{
    COMM_RX_START_BYTE_1,
    COMM_RX_START_BYTE_2,
    COMM_RX_LENGTH,
    COMM_RX_COMMAND_BYTE,
    COMM_RX_DATA
} serial_command_rx_state_t;

typedef struct {
    uint8_t length;
    uint8_t command_id;
    uint8_t data[64];
} serial_command_t;

static serial_command_rx_state_t  comm_rx_state;
static serial_command_t           comm_rx;
static uint8_t                    comm_rx_data_index;

int ledPin = 12;

void setup() {
  Serial.begin(9600);
  btSerial.begin(115200);
  pinMode(ledPin, OUTPUT);
  comm_rx_state = COMM_RX_START_BYTE_1;
  comm_rx_data_index = 0;
}

void loop() {
  bluetoothTick();
}

void bluetoothTick() {
  parsing();
}

String request = "12345";
void sendACK() {
  request = "";
  request += (char)0xF5;
  request += (char)0xAF;
  request += (char)0x01;
  request += (char)0xF0;
  request += (char)0x00;
  btSerial.print(request);
}

#define COMMAND_LED_ON_OFF    0x01


void uart_command_received(serial_command_t command){
    uint8_t command_id = command.command_id;
    switch(command_id) {
        case COMMAND_LED_ON_OFF:
            if (command.data[0] == 0) {
                Serial.println("need off");
                digitalWrite(ledPin, LOW);
            } else if (command.data[0] == 1) {
                Serial.println("need on");
                digitalWrite(ledPin, HIGH);
            }
            sendACK();
            break;
        default:
            break;
    }
}

void parsing() {
  
  if (btSerial.available() > 0) {
    uint8_t aByte = btSerial.read();      

    Serial.print("b: ");
    Serial.println(aByte,HEX);

    switch (comm_rx_state) {
        case COMM_RX_START_BYTE_1:
            if (aByte == 0xF5) {
                comm_rx_state = COMM_RX_START_BYTE_2;
                return;
            }
            break;
        case COMM_RX_START_BYTE_2:
            if (aByte == 0xAF) {
                comm_rx_state = COMM_RX_LENGTH;
                return;
            }
            break;
        case COMM_RX_LENGTH:
            if (aByte >= 0 && aByte <= 64) {
                comm_rx.length = aByte;
                comm_rx_state = COMM_RX_COMMAND_BYTE;
                return;
            }
            break;
        case COMM_RX_COMMAND_BYTE:
            comm_rx.command_id = aByte;
            comm_rx_data_index = 0;
            if (comm_rx.length > 0){
                comm_rx_state = COMM_RX_DATA;
                return;
            }
            break;
        case COMM_RX_DATA:
            comm_rx.data[comm_rx_data_index] = aByte;
            comm_rx_data_index++;
            if (comm_rx_data_index == comm_rx.length){
                uart_command_received(comm_rx);
            } else {
              return;
            }
            break;
    }
    comm_rx_state = COMM_RX_START_BYTE_1;
  }

}
