#include <MD_MAX72xx.h>
#include <SPI.h>

#define CS_PIN 53
#define MAX_DEVICES 8

// Try each type one at a time:
//MD_MAX72XX mx = MD_MAX72XX(MD_MAX72XX::FC16_HW, CS_PIN, MAX_DEVICES);
//MD_MAX72XX mx = MD_MAX72XX(MD_MAX72XX::GENERIC_HW, CS_PIN, MAX_DEVICES);
MD_MAX72XX mx = MD_MAX72XX(MD_MAX72XX::ICSTATION_HW, CS_PIN, MAX_DEVICES);

void setup() {
  mx.begin();
  mx.clear();
  
  // Draw a test pattern
  for (int i = 0; i < MAX_DEVICES; i++) {
    mx.setColumn(i * 8, 0xFF);  // Vertical line every 8 pixels
  }
}

void loop() {}