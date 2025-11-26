const int pinX = A0;
const int pinY = A1;
const int pinButton = 2;

int lastX = -1;
int lastY = -1;
int lastButton = -1;

const int deadzone = 5;
const unsigned long minInterval = 20;
unsigned long lastSend = 0;

void setup() {
  Serial.begin(115200);
  pinMode(pinButton, INPUT);
}

void loop() {
  int x = analogRead(pinX);
  int y = analogRead(pinY);
  int btn = (digitalRead(pinButton) == HIGH) ? 1 : 0;

  bool changed = false;
  if(abs(x - lastX) > deadzone || abs(y - lastY) > deadzone || btn != lastButton)
  {
    changed = true;
  }

  unsigned long now = millis();
  if(changed || (now - lastSend) > 100)
  {
    uint8_t packet[6];
    packet[0] = 0xAA;
    packet[1] = lowByte(x);
    packet[2] = highByte(x);
    packet[3] = lowByte(y);
    packet[4] = highByte(y);
    packet[5] = btn & 0x01;

    Serial.write(packet, sizeof(packet));

    lastX = x;
    lastY = y;
    lastButton = btn;
    lastSend = now;
  }
}
