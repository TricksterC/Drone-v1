// Flight controller: MPU6050 (Kalman) + Angle-mode PID + Motor mixing
// Pins: CH1 Roll -> 11, CH2 Pitch -> 10, CH3 Throttle -> 9, CH4 Yaw -> 8
// Motors: M1 4 (FR CW), M2 5 (RR CCW), M3 6 (RL CW), M4 7 (FL CCW)
// MPU6050: SDA -> A4, SCL -> A5, INT -> D2 (INT not used here)

#include <Wire.h>
#include <Servo.h>

// ===== Pins =====
const uint8_t CH1_PIN = 11;  // Roll (RC)
const uint8_t CH2_PIN = 10;  // Pitch (RC)
const uint8_t CH3_PIN = 9;   // Throttle (RC)
const uint8_t CH4_PIN = 8;   // Yaw (RC)

const uint8_t M1_PIN = 4;  // Front Right (CW)
const uint8_t M2_PIN = 5;  // Rear  Right (CCW)
const uint8_t M3_PIN = 6;  // Rear  Left  (CW)
const uint8_t M4_PIN = 7;  // Front Left (CCW)

Servo m1, m2, m3, m4;

// ===== ESC limits and RC midpoint =====
const int ESC_MIN = 1000;
const int ESC_MAX = 2000;
const int RC_MID = 1500;
const int THR_ACTIVATE = 1100;  // Minimum throttle to "arm" motors

// ===== RC -> angle mapping =====
const float MAX_ANGLE_DEG = 25.0; // stick full deflection -> ±25°

/* ===== MPU6050 / Kalman ===== */
#define MPU_ADDR 0x68
const float ACC_SENS  = 16384.0f; // ±2g
const float GYRO_SENS = 131.0f;   // ±250 °/s
const float RAD2DEG   = 57.29577951308232f;

long gyroBiasRawX = 0, gyroBiasRawY = 0, gyroBiasRawZ = 0;
float rollOffset = 0.0f, pitchOffset = 0.0f;
unsigned long lastMicros = 0;

// ---------- Simple 1D Kalman Filter class ----------
class Kalman1D {
public:
  Kalman1D() {
    Q_angle = 0.001f;
    Q_bias  = 0.003f;
    R_measure = 0.03f;
    angle = 0.0f; bias = 0.0f; P[0][0]=P[0][1]=P[1][0]=P[1][1]=0;
  }
  float update(float newAngle, float newRate, float dt) {
    rate = newRate - bias;
    angle += dt * rate;
    P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;
    float y = newAngle - angle;
    float S = P[0][0] + R_measure;
    float K0 = P[0][0] / S;
    float K1 = P[1][0] / S;
    angle += K0 * y;
    bias  += K1 * y;
    float P00 = P[0][0], P01 = P[0][1];
    P[0][0] -= K0 * P00;
    P[0][1] -= K0 * P01;
    P[1][0] -= K1 * P00;
    P[1][1] -= K1 * P01;
    return angle;
  }
  void setAngle(float a){ angle = a; }
private:
  float Q_angle, Q_bias, R_measure;
  float angle, bias, rate;
  float P[2][2];
};
Kalman1D kalmanRoll, kalmanPitch;

/* ===== PID (angle mode) ===== */
struct PID {
  float kp, ki, kd;
  float integrator;
  float lastErr;
  float outMin, outMax;
  PID(): kp(0), ki(0), kd(0), integrator(0), lastErr(0), outMin(-1000), outMax(1000) {}
  float update(float setpoint, float measurement, float dt) {
    float err = setpoint - measurement;
    integrator += err * dt;
    if (integrator * ki > outMax) integrator = outMax / max(ki, 1e-6f);
    if (integrator * ki < outMin) integrator = outMin / max(ki, 1e-6f);
    float deriv = (dt>0) ? (err - lastErr)/dt : 0.0f;
    lastErr = err;
    float out = kp * err + ki * integrator + kd * deriv;
    if (out > outMax) out = outMax;
    if (out < outMin) out = outMin;
    return out;
  }
};
PID pidRoll, pidPitch;

const float ROLL_KP = 10.0f, ROLL_KI = 0.02f, ROLL_KD = 1.0f;
const float PITCH_KP = 10.0f, PITCH_KI = 0.02f, PITCH_KD = 1.0f;

// ===== I2C helpers =====
void writeReg(uint8_t reg, uint8_t val){
  Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.write(val); Wire.endTransmission();
}
void readRegs(uint8_t reg, uint8_t count, uint8_t *buf){
  Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)count);
  uint8_t i = 0; while (Wire.available() && i < count) buf[i++] = Wire.read();
}
void initMPU(){
  Wire.begin();
  writeReg(0x6B, 0x00);
  delay(50);
  writeReg(0x1A, 0x03);
  writeReg(0x1B, 0x00);
  writeReg(0x1C, 0x00);
  writeReg(0x19, 0x04);
  delay(50);
}
void readRawAccelGyro(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz){
  uint8_t buf[14]; readRegs(0x3B, 14, buf);
  ax = (int16_t)((buf[0]<<8)|buf[1]);
  ay = (int16_t)((buf[2]<<8)|buf[3]);
  az = (int16_t)((buf[4]<<8)|buf[5]);
  gx = (int16_t)((buf[8]<<8)|buf[9]);
  gy = (int16_t)((buf[10]<<8)|buf[11]);
  gz = (int16_t)((buf[12]<<8)|buf[13]);
}

// gyro calibration
void calibrateGyro(int samples = 800){
  long sx=0, sy=0, sz=0;
  for(int i=0;i<samples;i++){
    int16_t ax,ay,az,gx,gy,gz;
    readRawAccelGyro(ax,ay,az,gx,gy,gz);
    sx += gx; sy += gy; sz += gz; delay(2);
  }
  gyroBiasRawX = sx / samples;
  gyroBiasRawY = sy / samples;
  gyroBiasRawZ = sz / samples;
}

// accel offsets capture
void captureAccelOffsets(int samples = 300){
  double rsum=0, psum=0;
  for(int i=0;i<samples;i++){
    int16_t axr, ayr, azr, gxr, gyr, gzr;
    readRawAccelGyro(axr,ayr,azr,gxr,gyr,gzr);
    float ax = axr / ACC_SENS;
    float ay = ayr / ACC_SENS;
    float az = azr / ACC_SENS;
    rsum += atan2(ay, az)*RAD2DEG;
    psum += atan2(-ax, sqrt(ay*ay+az*az))*RAD2DEG;
    delay(5);
  }
  rollOffset = rsum / samples;
  pitchOffset = psum / samples;
}

// ===== RC Read =====
int readChannelUs(uint8_t pin){
  unsigned long us = pulseIn(pin, HIGH, 30000UL);
  if(us==0) return RC_MID;
  return (int)constrain(us, ESC_MIN, ESC_MAX);
}

void stopMotors(){
  m1.writeMicroseconds(ESC_MIN);
  m2.writeMicroseconds(ESC_MIN);
  m3.writeMicroseconds(ESC_MIN);
  m4.writeMicroseconds(ESC_MIN);
}

void setup(){
  Serial.begin(115200);
  while(!Serial){;}

  m1.attach(M1_PIN); m2.attach(M2_PIN); m3.attach(M3_PIN); m4.attach(M4_PIN);
  stopMotors();

  pinMode(CH1_PIN, INPUT); pinMode(CH2_PIN, INPUT);
  pinMode(CH3_PIN, INPUT); pinMode(CH4_PIN, INPUT);

  initMPU(); delay(100);
  calibrateGyro(800);
  captureAccelOffsets(300);

  kalmanRoll.setAngle(0.0f); kalmanPitch.setAngle(0.0f);

  pidRoll.kp=ROLL_KP; pidRoll.ki=ROLL_KI; pidRoll.kd=ROLL_KD;
  pidPitch.kp=PITCH_KP; pidPitch.ki=PITCH_KI; pidPitch.kd=PITCH_KD;
  pidRoll.outMin=-400; pidRoll.outMax=400;
  pidPitch.outMin=-400; pidPitch.outMax=400;

  lastMicros = micros();
}

void loop(){
  int thrUs = readChannelUs(CH3_PIN);
  int rollUs = readChannelUs(CH1_PIN);
  int pitchUs = readChannelUs(CH2_PIN);
  int yawUs = readChannelUs(CH4_PIN);

  // --- Safety check ---
  if(thrUs < THR_ACTIVATE){ 
    stopMotors();
    return;  // Skip all MPU/PID/mixing if below throttle
  }

  // map RC sticks to setpoints
  float rcRollNorm = constrain((float)(rollUs-RC_MID)/500.0f, -1.0f, 1.0f);
  float rcPitchNorm = constrain((float)(pitchUs-RC_MID)/500.0f, -1.0f, 1.0f);
  float setRollDeg = rcRollNorm * MAX_ANGLE_DEG;
  float setPitchDeg = rcPitchNorm * MAX_ANGLE_DEG;

  // read MPU
  int16_t axr,ayr,azr,gxr,gyr,gzr;
  readRawAccelGyro(axr,ayr,azr,gxr,gyr,gzr);
  float ax=axr/ACC_SENS, ay=ayr/ACC_SENS, az=azr/ACC_SENS;
  float gx=(gxr-gyroBiasRawX)/GYRO_SENS;
  float gy=(gyr-gyroBiasRawY)/GYRO_SENS;

  // accel angles + offsets
  float accRoll  = atan2(ay,az)*RAD2DEG - rollOffset;
  float accPitch = atan2(-ax,sqrt(ay*ay+az*az))*RAD2DEG - pitchOffset;

  // dt
  unsigned long now = micros();
  float dt = (now - lastMicros)/1000000.0f;
  if(dt<=0||dt>0.1f) dt=0.005f;
  lastMicros=now;

  float kalRoll = kalmanRoll.update(accRoll,gx,dt);
  float kalPitch = kalmanPitch.update(accPitch,gy,dt);

  // PID
  float pidRollOut = pidRoll.update(setRollDeg, kalRoll, dt);
  float pidPitchOut = pidPitch.update(setPitchDeg, kalPitch, dt);

  int yawAdj = map(constrain(yawUs,1000,2000),1000,2000,-120,120);
  int baseThr = constrain(thrUs, ESC_MIN, ESC_MAX);

  // Motor mixing
  int m1_speed = baseThr - (int)pidRollOut - (int)pidPitchOut - yawAdj;
  int m2_speed = baseThr - (int)pidRollOut + (int)pidPitchOut + yawAdj;
  int m3_speed = baseThr + (int)pidRollOut + (int)pidPitchOut - yawAdj;
  int m4_speed = baseThr + (int)pidRollOut - (int)pidPitchOut + yawAdj;

  m1.writeMicroseconds(constrain(m1_speed, ESC_MIN, ESC_MAX));
  m2.writeMicroseconds(constrain(m2_speed, ESC_MIN, ESC_MAX));
  m3.writeMicroseconds(constrain(m3_speed, ESC_MIN, ESC_MAX));
  m4.writeMicroseconds(constrain(m4_speed, ESC_MIN, ESC_MAX));

  delay(2);
}