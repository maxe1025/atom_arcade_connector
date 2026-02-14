const int pinX = A0;
const int pinY = A1;
const int pinButton = 2;

const int centerX = 512;
const int centerY = 512;
const int centerDeadzone = 20;
const int changeDeadzone = 5;

int lastX = -1;
int lastY = -1;
int lastButton = -1;

const unsigned long minInterval = 20;
unsigned long lastSend = 0;

void setup() {
  Serial.begin(115200);
  pinMode(pinButton, INPUT_PULLUP);
}

void loop() {
  int x = analogRead(pinX);
  int y = analogRead(pinY);
  int btn = (digitalRead(pinButton) == LOW) ? 1 : 0;

  if (abs(x - centerX) < centerDeadzone) x = centerX;
  if (abs(y - centerY) < centerDeadzone) y = centerY;

  bool changed = false;
  if(abs(x - lastX) > changeDeadzone || 
      abs(y - lastY) > changeDeadzone ||
      btn != lastButton)
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
    packet[5] = btn & 0x01;
    packet[6] = (packet[1] + packet[2] + packet[3] + packet[4] + packet[5]) & 0xFF;

    Serial.write(packet, sizeof(packet));

    lastX = x;
    lastY = y;
    lastButton = btn;
    lastSend = now;
  }
}
