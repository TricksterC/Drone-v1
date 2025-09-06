#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

const int CALIBRATION_SAMPLES = 1000;

float accelX_max_g, accelX_min_g;

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println("MPU-6050 Accelerometer X-Axis Calibration");
  
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  // --- Step 1: Calibrate +X Axis ---
  Serial.println("\nPlace the sensor with the +X axis pointing straight up.");
  Serial.println("Press any key to continue...");
  while (Serial.available() == 0) {
    delay(100);
  }
  // Clear the entire serial buffer to fix the loop issue
  while (Serial.available() > 0) {
    Serial.read();
  }

  Serial.println("Calibrating +X Axis...");
  float accelX_sum = 0;
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    accelX_sum += a.acceleration.x;
    delay(2);
  }
  accelX_max_g = accelX_sum / CALIBRATION_SAMPLES;
  Serial.print("Average +X reading: ");
  Serial.print(accelX_max_g, 4);
  Serial.println(" m/s^2");

  // --- Step 2: Calibrate -X Axis ---
  Serial.println("\nPlace the sensor with the -X axis pointing straight up.");
  Serial.println("Press any key to continue...");
  while (Serial.available() == 0) {
    delay(100);
  }
  // Clear the entire serial buffer again
  while (Serial.available() > 0) {
    Serial.read();
  }

  Serial.println("Calibrating -X Axis...");
  accelX_sum = 0; // Reset sum for -X
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    accelX_sum += a.acceleration.x;
    delay(2);
  }
  accelX_min_g = accelX_sum / CALIBRATION_SAMPLES;
  Serial.print("Average -X reading: ");
  Serial.print(accelX_min_g, 4);
  Serial.println(" m/s^2");

  // --- Calculate and Print Offset and Scale Factor ---
  float accelX_offset = (accelX_max_g + accelX_min_g) / 2.0;
  float accelX_scale = 9.81 / (accelX_max_g - accelX_offset);

  Serial.println("\nCalibration complete!");
  Serial.print("Accelerometer X Offset: ");
  Serial.println(accelX_offset, 4);
  Serial.print("Accelerometer X Scale Factor: ");
  Serial.println(accelX_scale, 4);
  Serial.println("Use these values in your main sketch to correct the X-axis readings.");
}

void loop() {
  // Loop is not used in this calibration sketch
  delay(1000);
}
