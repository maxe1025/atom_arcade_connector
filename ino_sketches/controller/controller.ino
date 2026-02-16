// Joystick pins
const int pinX = A0;
const int pinY = A1;

// Button pins
const int pinButtonA = 2;
const int pinButtonB = 3;
const int pinButtonX = 4;
const int pinButtonY = 5;
const int pinButtonLB = 6;
const int pinButtonRB = 7;
const int pinButtonStart = 8;

// Button bit positions
const uint8_t BTN_A     = 0b00000001;
const uint8_t BTN_B     = 0b00000010;
const uint8_t BTN_X     = 0b00000100;
const uint8_t BTN_Y     = 0b00001000;
const uint8_t BTN_LB    = 0b00010000;
const uint8_t BTN_RB    = 0b00100000;
const uint8_t BTN_START = 0b01000000;

// Joystick calibration
const int centerX = 512;
const int centerY = 512;
const int centerDeadzone = 20;
const int changeDeadzone = 5;

int lastX = -1;
int lastY = -1;
uint8_t lastButtons = 0;

const unsigned long minInterval = 20;
unsigned long lastSend = 0;

void setup() {
  Serial.begin(115200);

  pinMode(pinButtonA, INPUT_PULLUP);
  pinMode(pinButtonB, INPUT_PULLUP);
  pinMode(pinButtonX, INPUT_PULLUP);
  pinMode(pinButtonY, INPUT_PULLUP);
  pinMode(pinButtonLB, INPUT_PULLUP);
  pinMode(pinButtonRB, INPUT_PULLUP);
  pinMode(pinButtonStart, INPUT_PULLUP);
}

void loop() {
  int x = analogRead(pinX);
  int y = analogRead(pinY);

  if (abs(x - centerX) < centerDeadzone) x = centerX;
  if (abs(y - centerY) < centerDeadzone) y = centerY;

  uint8_t buttons = 0;
  if (digitalRead(pinButtonA) == LOW)     buttons |= BTN_A;
  if (digitalRead(pinButtonB) == LOW)     buttons |= BTN_B;
  if (digitalRead(pinButtonX) == LOW)     buttons |= BTN_X;
  if (digitalRead(pinButtonY) == LOW)     buttons |= BTN_Y;
  if (digitalRead(pinButtonLB) == LOW)    buttons |= BTN_LB;
  if (digitalRead(pinButtonRB) == LOW)    buttons |= BTN_RB;
  if (digitalRead(pinButtonStart) == LOW) buttons |= BTN_START;

  bool changed = false;
  if(abs(x - lastX) > changeDeadzone || 
      abs(y - lastY) > changeDeadzone ||
      buttons != lastButtons)
  {
    changed = true;
  }

  unsigned long now = millis();
  if(changed || (now - lastSend) > 100)
  {
    uint8_t packet[7];
    packet[0] = 0xAA;
    packet[1] = lowByte(x);
    packet[2] = highByte(x);
    packet[3] = lowByte(y);
    packet[4] = highByte(y);
    packet[5] = buttons;
    packet[6] = (packet[1] + packet[2] + packet[3] + packet[4] + packet[5]) & 0xFF;

    Serial.write(packet, sizeof(packet));

    lastX = x;
    lastY = y;
    lastButtons = buttons;
    lastSend = now;
  }
}
