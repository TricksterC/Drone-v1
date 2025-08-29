// WORKING CODE OF TRANSMITTER 
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <nRF24L01.h>
#include <RF24.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

RF24 radio(9, 10);  // CE, CSN
const byte address[6] = "00001";

#define MOV_X A0
#define MOV_Y A1
#define MOV_SW 3
#define CAM_X A2
#define CAM_Y A3
#define CAM_SW 4
#define TPB3 A6
#define TPB4 A7
#define ARM_SWITCH 6
#define ALT_SWITCH 5
#define YAW_L 7
#define YAW_R 8
#define BUZZER 2

struct ControlData {
  int throttle;
  int yaw;
  int pitch;
  int roll;
  int camX;
  int camY;
  bool armed;
  bool altHold;
  bool lsw;
  bool rsw;
};
ControlData data;

bool lastArmState = false;
bool lastAltState = false;
int throttle = 0;
int yawVal = 512;
int camXState = 0, camYState = 0;
String camDir = "Center", prevCamDir = "Center";

unsigned long lastSendSuccess = 0;
unsigned long lastSendAttempt = 0;
bool isConnected = false;
bool lowBatteryAlerted = false;

void shortBeep(int freq = 1000, int dur = 60) {
  tone(BUZZER, freq, dur);
  delay(dur + 10);
  noTone(BUZZER);
}

void startupMelody() {
  shortBeep(1000, 80);
  shortBeep(1500, 80);
  shortBeep(2000, 80);
}

long readVcc() {
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2);
  ADCSRA |= _BV(ADSC);
  while (bit_is_set(ADCSRA, ADSC));
  return 1125300L / ADC;
}

void drawBatteryIcon(int x, int y, int level) {
  display.drawRect(x, y, 18, 8, SSD1306_WHITE);
  display.drawRect(x + 18, y + 2, 2, 4, SSD1306_WHITE);
  int fillBars = map(level, 0, 100, 0, 4);
  for (int i = 0; i < fillBars; i++) {
    display.fillRect(x + 2 + i * 4, y + 2, 3, 4, SSD1306_WHITE);
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(MOV_SW, INPUT_PULLUP);
  pinMode(CAM_SW, INPUT_PULLUP);
  pinMode(YAW_L, INPUT_PULLUP);
  pinMode(YAW_R, INPUT_PULLUP);
  pinMode(ARM_SWITCH, INPUT_PULLUP);
  pinMode(ALT_SWITCH, INPUT_PULLUP);
  pinMode(BUZZER, OUTPUT);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) while (1);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Booting...");
  display.display();

  radio.begin();
  radio.setAutoAck(true);
  radio.setRetries(5, 15);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(76);
  radio.openWritingPipe(address);
  radio.stopListening();

  startupMelody();
}

void loop() {
  display.setTextSize(1);

  // Read Switches
  data.armed = !digitalRead(ARM_SWITCH);
  data.altHold = !digitalRead(ALT_SWITCH);

  // ========== Camera Joystick ==========

  int camX = analogRead(CAM_X);
  int camY = analogRead(CAM_Y);

  // Add deadzone for noise filtering
  const int deadzone = 50;
  int newCamXState = (camX < 512 - deadzone) ? -1 : (camX > 512 + deadzone) ? 1 : 0;
  int newCamYState = (camY < 512 - deadzone) ? 1 : (camY > 512 + deadzone) ? -1 : 0;

  // Only update if state has actually changed
  if (newCamXState != camXState || newCamYState != camYState) {
    camXState = newCamXState;
    camYState = newCamYState;

    if (camXState == -1) prevCamDir = "Left";
    else if (camXState == 1) prevCamDir = "Right";
    else if (camYState == 1) prevCamDir = "Top";
    else if (camYState == -1) prevCamDir = "Bottom";
  }

  if (camXState == 0 && camYState == 0) {
    camDir = prevCamDir;  // Keep showing last direction
  } else {
    camDir = prevCamDir;  // Still update if moved
  }

  data.camX = camXState;
  data.camY = camYState;

  // Roll & Pitch
  data.roll = map(analogRead(MOV_X), 0, 1023, 512, -511);
  data.pitch = map(analogRead(MOV_Y), 0, 1023, -512, 511);
  if (abs(data.pitch) < 22) data.pitch = 0;
  if (abs(data.roll) < 22) data.roll = 0;

  // Handle UP Button (A6)
  if (analogRead(TPB3) < 200) {
    if (!data.armed) {
      display.clearDisplay();
      display.setTextSize(2);
      display.setCursor(10, 25); display.println("Switch");
      display.setCursor(10, 45); display.println("ON ARM");
      display.display();
      tone(BUZZER, 400, 300); delay(600); noTone(BUZZER);
      display.setTextSize(1);
      return;
    } else if (data.altHold) {
      display.clearDisplay();
      display.setCursor(10, 30); display.println("Altitude is HOLD ON");
      display.display();
      tone(BUZZER, 500, 200); delay(500); noTone(BUZZER);
      return;
    } else {
      throttle += 50;
      throttle = constrain(throttle, 0, 1023);
      shortBeep();
      delay(150);
    }
  }

  // Handle DOWN Button (A7)
  if (analogRead(TPB4) < 200) {
    if (!data.armed) {
      display.clearDisplay();
      display.setTextSize(2);
      display.setCursor(10, 25); display.println("Switch");
      display.setCursor(10, 45); display.println("ON ARM");
      display.display();
      tone(BUZZER, 400, 300); delay(600); noTone(BUZZER);
      display.setTextSize(1);
      return;
    } else if (data.altHold) {
      display.clearDisplay();
      display.setCursor(10, 30); display.println("Altitude is HOLD ON");
      display.display();
      tone(BUZZER, 500, 200); delay(500); noTone(BUZZER);
      return;
    } else {
      throttle -= 50;
      throttle = constrain(throttle, 0, 1023);
      shortBeep();
      delay(150);
    }
  }

  data.throttle = throttle;

  // YAW
  bool yawLeft = digitalRead(YAW_L) == LOW;
  bool yawRight = digitalRead(YAW_R) == LOW;
  if (yawLeft) {
    yawVal -= 50;
    yawVal = constrain(yawVal, 0, 1023);
    shortBeep();
    delay(150);
  } else if (yawRight) {
    yawVal += 50;
    yawVal = constrain(yawVal, 0, 1023);
    shortBeep();
    delay(150);
  } else {
    yawVal = 512;
  }
  data.yaw = yawVal;

  // Switch changes
  if (data.armed != lastArmState) {
    shortBeep(data.armed ? 1600 : 600, 100);
    lastArmState = data.armed;
  }
  if (data.altHold != lastAltState) {
    shortBeep(data.altHold ? 1400 : 800, 100);
    lastAltState = data.altHold;
  }

  // LSW/RSW
  bool currentLSW = !digitalRead(CAM_SW);
  bool currentRSW = !digitalRead(MOV_SW);
  if (currentLSW != data.lsw || currentRSW != data.rsw) shortBeep();
  data.lsw = currentLSW;
  data.rsw = currentRSW;

  // Send data
  bool sendResult = radio.write(&data, sizeof(data));
  unsigned long now = millis();
  if (sendResult) lastSendSuccess = now;
  isConnected = (now - lastSendSuccess < 1000);

  // Battery check
  long vcc = readVcc();
  float voltage = vcc / 1000.0;
  int batteryPct = (voltage >= 4.2) ? 100 : (voltage <= 3.3) ? 0 : (int)((voltage - 3.3) / (4.2 - 3.3) * 100.0);

  if (batteryPct <= 20 && !lowBatteryAlerted) {
    tone(BUZZER, 600, 300); delay(400); noTone(BUZZER);
    lowBatteryAlerted = true;
  }
  if (batteryPct > 20) lowBatteryAlerted = false;

  // DISPLAY
  display.clearDisplay();
  drawBatteryIcon(SCREEN_WIDTH - 22, 0, batteryPct);
  display.setCursor(0, 0);
  display.print("UP: "); display.print(throttle);
  display.print(" DN: "); display.println(1023 - throttle);
  display.print("YAW: "); display.println(data.yaw);

  if (data.pitch > 0) display.print("FWD: "), display.println(data.pitch);
  else if (data.pitch < 0) display.print("BWD: "), display.println(-data.pitch);
  else display.println("FWD/BWD: 0");

  if (data.roll < 0) display.print("LEFT: "), display.println(-data.roll);
  else if (data.roll > 0) display.print("RGT: "), display.println(data.roll);
  else display.println("LEFT/RGT: 0");

  display.print("CAM: ");
  display.print((camXState == 0 && camYState == 0) ? "Center" : camDir);
  display.print(" ("); display.print(prevCamDir); display.println(")");

  display.print("ARM: "); display.print(data.armed ? "YES" : "NO");
  display.print(" ALT: "); display.println(data.altHold ? "YES" : "NO");

  display.print("LSW: "); display.print(data.lsw);
  display.print(" RSW: "); display.print(data.rsw);

  display.setCursor(0, 56);
  display.print("STATUS: "); display.println(isConnected ? "PAIRED" : "UNPAIRED");
  display.display();
}