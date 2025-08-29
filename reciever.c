#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>

// ================= Radio Setup =================
RF24 radio(9, 10);  // CE, CSN
const byte address[6] = "00001";

// Data struct (MUST match transmitter)
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

// ================= Channels =================
Servo ch1; // Roll
Servo ch2; // Pitch
Servo ch3; // Throttle
Servo ch4; // Yaw

#define CH1_PIN 7   // Roll
#define CH2_PIN 6   // Pitch
#define CH3_PIN 5   // Throttle
#define CH4_PIN 3   // Yaw

// RC Pulse Range
#define RC_MIN 1000
#define RC_MAX 2000
#define RC_MID 1500

// Failsafe timeout (ms)
#define FAILSAFE_TIMEOUT 500  

unsigned long lastSignalTime = 0;

void setup() {
  Serial.begin(9600);

  // Attach servos (as RC PWM output generator)
  ch1.attach(CH1_PIN);
  ch2.attach(CH2_PIN);
  ch3.attach(CH3_PIN);
  ch4.attach(CH4_PIN);

  // Start radio
  radio.begin();
  radio.setAutoAck(true);
  radio.setRetries(5, 15);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(76);
  radio.openReadingPipe(0, address);
  radio.startListening();

  Serial.println("Receiver Started...");
}

void loop() {
  if (radio.available()) {
    radio.read(&data, sizeof(data));
    lastSignalTime = millis();  // refresh signal timer
  }

  int rollPWM, pitchPWM, throttlePWM, yawPWM;

  // ========== FAILSAFE CHECK ==========
  if (millis() - lastSignalTime > FAILSAFE_TIMEOUT) {
    // Lost signal → cut throttle
    rollPWM = RC_MID;
    pitchPWM = RC_MID;
    yawPWM = RC_MID;
    throttlePWM = RC_MIN;
    Serial.println("FAILSAFE TRIGGERED!");
  }
  else {
    // ========== ARM/DISARM CHECK ==========
    if (data.armed) {
      // Map values from transmitter to RC PWM range
      rollPWM     = map(data.roll,   -512, 512, RC_MIN, RC_MAX);
      pitchPWM    = map(data.pitch,  -512, 512, RC_MIN, RC_MAX);
      throttlePWM = map(data.throttle, 0, 1023, RC_MIN, RC_MAX);
      yawPWM      = map(data.yaw,      0, 1023, RC_MIN, RC_MAX);
    } else {
      // Disarmed → force throttle low, others neutral
      rollPWM = RC_MID;
      pitchPWM = RC_MID;
      yawPWM = RC_MID;
      throttlePWM = RC_MIN;
    }
  }

  // Write to output pins
  ch1.writeMicroseconds(rollPWM);
  ch2.writeMicroseconds(pitchPWM);
  ch3.writeMicroseconds(throttlePWM);
  ch4.writeMicroseconds(yawPWM);

  // Debugging
  Serial.print("ARMED: "); Serial.print(data.armed);
  Serial.print("  ROLL: "); Serial.print(rollPWM);
  Serial.print("  PITCH: "); Serial.print(pitchPWM);
  Serial.print("  THROTTLE: "); Serial.print(throttlePWM);
  Serial.print("  YAW: "); Serial.print(yawPWM);
  Serial.print("  Signal age: "); Serial.println(millis() - lastSignalTime);
}