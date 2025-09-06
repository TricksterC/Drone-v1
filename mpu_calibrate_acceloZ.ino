#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

const int CALIBRATION_SAMPLES = 1000;

float accelZ_max_g, accelZ_min_g;

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println("MPU-6050 Accelerometer Z-Axis Calibration");
  
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  // --- Step 1: Calibrate +Z Axis ---
  Serial.println("\nPlace the sensor with the +Z axis pointing straight up.");
  Serial.println("Press any key to continue...");
  while (Serial.available() == 0) {
    delay(100);
  }
  while (Serial.available() > 0) {
    Serial.read();
  }

  Serial.println("Calibrating +Z Axis...");
  float accelZ_sum = 0;
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    accelZ_sum += a.acceleration.z;
    delay(2);
  }
  accelZ_max_g = accelZ_sum / CALIBRATION_SAMPLES;
  Serial.print("Average +Z reading: ");
  Serial.print(accelZ_max_g, 4);
  Serial.println(" m/s^2");

  // --- Step 2: Calibrate -Z Axis ---
  Serial.println("\nPlace the sensor with the -Z axis pointing straight up.");
  Serial.println("Press any key to continue...");
  while (Serial.available() == 0) {
    delay(100);
  }
  while (Serial.available() > 0) {
    Serial.read();
  }

  Serial.println("Calibrating -Z Axis...");
  accelZ_sum = 0;
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    accelZ_sum += a.acceleration.z;
    delay(2);
  }
  accelZ_min_g = accelZ_sum / CALIBRATION_SAMPLES;
  Serial.print("Average -Z reading: ");
  Serial.print(accelZ_min_g, 4);
  Serial.println(" m/s^2");

  // --- Calculate and Print Offset and Scale Factor ---
  float accelZ_offset = (accelZ_max_g + accelZ_min_g) / 2.0;
  float accelZ_scale = 9.81 / (accelZ_max_g - accelZ_offset);

  Serial.println("\nCalibration complete!");
  Serial.print("Accelerometer Z Offset: ");
  Serial.println(accelZ_offset, 4);
  Serial.print("Accelerometer Z Scale Factor: ");
  Serial.println(accelZ_scale, 4);
  Serial.println("Use these values in your main sketch to correct the Z-axis readings.");
}

void loop() {
  delay(1000);
}
