#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

const int CALIBRATION_SAMPLES = 1000;

float accelY_max_g, accelY_min_g;

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println("MPU-6050 Accelerometer Y-Axis Calibration");
  
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  // --- Step 1: Calibrate +Y Axis ---
  Serial.println("\nPlace the sensor with the +Y axis pointing straight up.");
  Serial.println("Press any key to continue...");
  while (Serial.available() == 0) {
    delay(100);
  }
  while (Serial.available() > 0) {
    Serial.read();
  }

  Serial.println("Calibrating +Y Axis...");
  float accelY_sum = 0;
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    accelY_sum += a.acceleration.y;
    delay(2);
  }
  accelY_max_g = accelY_sum / CALIBRATION_SAMPLES;
  Serial.print("Average +Y reading: ");
  Serial.print(accelY_max_g, 4);
  Serial.println(" m/s^2");

  // --- Step 2: Calibrate -Y Axis ---
  Serial.println("\nPlace the sensor with the -Y axis pointing straight up.");
  Serial.println("Press any key to continue...");
  while (Serial.available() == 0) {
    delay(100);
  }
  while (Serial.available() > 0) {
    Serial.read();
  }

  Serial.println("Calibrating -Y Axis...");
  accelY_sum = 0;
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    accelY_sum += a.acceleration.y;
    delay(2);
  }
  accelY_min_g = accelY_sum / CALIBRATION_SAMPLES;
  Serial.print("Average -Y reading: ");
  Serial.print(accelY_min_g, 4);
  Serial.println(" m/s^2");

  // --- Calculate and Print Offset and Scale Factor ---
  float accelY_offset = (accelY_max_g + accelY_min_g) / 2.0;
  float accelY_scale = 9.81 / (accelY_max_g - accelY_offset);

  Serial.println("\nCalibration complete!");
  Serial.print("Accelerometer Y Offset: ");
  Serial.println(accelY_offset, 4);
  Serial.print("Accelerometer Y Scale Factor: ");
  Serial.println(accelY_scale, 4);
  Serial.println("Use these values in your main sketch to correct the Y-axis readings.");
}

void loop() {
  delay(1000);
}
