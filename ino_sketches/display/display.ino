#include <MD_Parola.h>
#include <MD_MAX72xx.h>
#include <SPI.h>

/* Pin configuration

Arduino Uno → MAX7219
──────────────────────
5V   → VCC
GND  → GND
D11  → DIN  (MOSI)
D13  → CLK  (SCK)
D10  → CS   (SS)
*/

// Config for Arduino Mega. Chnage for UNO!!!
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
#define MAX_DEVICES 8
#define CS_PIN 10

MD_Parola display = MD_Parola(HARDWARE_TYPE, CS_PIN, MAX_DEVICES);

const uint8_t CMD_HEADER = 0xBB;
const uint8_t CMD_SHOW_TEXT = 0x01;
const uint8_t CMD_CLEAR = 0x02;
const uint8_t CMD_SET_BRIGHTNESS = 0x03;

uint8_t buffer[256];
int bufferIndex = 0;
bool receivingMessage = false;

void setup() {
  Serial.begin(115200);
  
  display.begin();
  display.setIntensity(1);
  display.displayClear();
  display.setZoneEffect(0, true, PA_FLIP_UD);
  display.setZoneEffect(0, true, PA_FLIP_LR);
  display.setTextAlignment(PA_CENTER);
  display.print("Atom Arcade");
}

void loop() {
  if (Serial.available() > 0) {
    processSerialData();
  }

  display.displayAnimate();
}

/*
 * Process incoming serial data
 * Packet format: [HEADER][COMMAND][LENGTH][DATA][CHECKSUM]
 */
void processSerialData() {
  while (Serial.available() > 0) {
    uint8_t byte = Serial.read();
    
    if (!receivingMessage) {
      if (byte == CMD_HEADER) {
        receivingMessage = true;
        bufferIndex = 0;
      }
      continue;
    }
    
    buffer[bufferIndex++] = byte;
    
    if (bufferIndex >= 2) {
      uint8_t command = buffer[0];
      uint8_t length = buffer[1];
      
      if (bufferIndex >= (2 + length + 1)) {
        uint8_t checksum = 0;
        for (int i = 0; i < (2 + length); i++) {
          checksum += buffer[i];
        }
        checksum &= 0xFF;
        
        if (checksum == buffer[2 + length]) {
          executeCommand(command, &buffer[2], length);
        }
        
        receivingMessage = false;
        bufferIndex = 0;
      }
      
      if (bufferIndex >= 255) {
        receivingMessage = false;
        bufferIndex = 0;
      }
    }
  }
}

/*
 * Execute display commands
 */
void executeCommand(uint8_t command, uint8_t* data, uint8_t length) {
  switch (command) {
    case CMD_SHOW_TEXT: {
      char text[128];
      memcpy(text, data, length);
      text[length] = '\0';
      
      display.displayClear();
      display.print(text);
      break;
    }
    
    case CMD_CLEAR:
      display.displayClear();
      break;
    
    case CMD_SET_BRIGHTNESS:
      if (length >= 1) {
        uint8_t brightness = data[0];
        if (brightness <= 15) {
          display.setIntensity(brightness);
        }
      }
      break;
  }
}