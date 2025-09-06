#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

float gyroX_offset, gyroY_offset, gyroZ_offset;
const int CALIBRATION_SAMPLES = 1000;

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println("MPU-6050 Gyroscope Calibration");
  Serial.println("Place the sensor on a level, stable surface and do not move it.");
  delay(2000);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  Serial.println("Calibrating...");

  
  float gyroX_sum = 0, gyroY_sum = 0, gyroZ_sum = 0;

  // Take a large number of readings for averaging
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    gyroX_sum += g.gyro.x;
    gyroY_sum += g.gyro.y;
    gyroZ_sum += g.gyro.z;

    delay(2);
  }

  gyroX_offset = gyroX_sum / CALIBRATION_SAMPLES;
  gyroY_offset = gyroY_sum / CALIBRATION_SAMPLES;
  gyroZ_offset = gyroZ_sum / CALIBRATION_SAMPLES;

  Serial.println("Calibration complete!");
  Serial.print("Gyro X Offset: ");
  Serial.println(gyroX_offset, 4);
  Serial.print("Gyro Y Offset: ");
  Serial.println(gyroY_offset, 4);
  Serial.print("Gyro Z Offset: ");
  Serial.println(gyroZ_offset, 4);
}

void loop() {
  delay(1000);
}
