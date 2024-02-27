#include "Wire.h"
#include "I2Cdev.h"
#include "MPU9250.h"
#include "math.h"
#include <PID_v1.h>

MPU9250 mpu;

// Kalman filter parameters and variables
float Q_angle = 0.001;
float Q_bias = 0.003;
float R_measure = 0.03;
//float angleXZ = 0; // The angle calculated by the Kalman filter
float biasXZ = 0; 
float rateXZ = 0; 
float P[2][2] = {0};

// Magnetometer data
int16_t magX, magY, magZ;
float pitch, azimuth;

// Calibration offsets
int16_t gyroX_offset, gyroY_offset, gyroZ_offset;
int16_t accX_offset, accY_offset, accZ_offset;

int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;
float accAngleXZ, accAngleYZ, gyroRateX, gyroRateY, gyroAngleX, angleXZ, angleYZ;
unsigned long currentTime, previousTime;
float elapsedTime, filterCoefficient = 0.5;

// Motor control pins
int enA = 10;
int in1 = 8;
int in2 = 9;
int enB = 5;
int in3 = 6;
int in4 = 7;

// PID variables
double input, output, setpoint;
double kp = 43; // Set your PID constants
double ki = 34;
double kd = 0.74;
PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

void setup() {
  Wire.begin();
  Serial.begin(9600);
  mpu.initialize();

  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println("MPU9250 connection successful");
  } else {
    Serial.println("MPU9250 connection failed");
    while(1);
  }
  calibrateSensors();
  previousTime = micros();

  // Motor control pin setup
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Initialize the PID controller
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255); // Set the output limits for motor control
  setpoint = 0.0; // Initialize the setpoint to your desired angle
}

void loop() {
  currentTime = micros();
  elapsedTime = (currentTime - previousTime) / 1000000.0;
  previousTime = currentTime;

  mpu.getMotion9(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ, &magX, &magY, &magZ);


  accX -= accX_offset;
  accY -= accY_offset;
  accZ -= accZ_offset;
  gyroX -= gyroX_offset;
  gyroY -= gyroY_offset;
  gyroZ -= gyroZ_offset;


    // Initial pitch estimation from accelerometer
    float pitchAcc = atan2(accY, sqrt(accX*accX + accZ*accZ)) * RAD_TO_DEG;

    // Kalman filter implementation for pitch
    gyroRateX = gyroX / 131.0; // Assuming gyro is set to 250dps
    rateXZ = gyroRateX - biasXZ;
    pitch += elapsedTime * rateXZ;

    // Update the error covariance matrix
    P[0][0] += elapsedTime * (elapsedTime*P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= elapsedTime * P[1][1];
    P[1][0] -= elapsedTime * P[1][1];
    P[1][1] += Q_bias * elapsedTime;

    // Calculate Kalman gain
    float S = P[0][0] + R_measure;
    float K[2] = {P[0][0] / S, P[1][0] / S};

    // Calculate angle and bias - Update estimate with measurement zk (pitchAcc)
    float y = pitchAcc - pitch;
    pitch += K[0] * y;
    biasXZ += K[1] * y;

    // Update error covariance matrix
    P[0][0] -= K[0] * P[0][0];
    P[0][1] -= K[0] * P[0][1];
    P[1][0] -= K[1] * P[0][0];
    P[1][1] -= K[1] * P[0][1];


  // Use the angleXZ as the input for the PID controller
  input = pitch;

  // Compute PID output
  myPID.Compute();

  // Calculate motor speeds
  int motorSpeedA = int(output);
  int motorSpeedB = int(output);

  // Ensure the motor speeds are within valid limits
  if (motorSpeedA > 255) motorSpeedA = 255;
  else if (motorSpeedA < -255) motorSpeedA = -255;

  if (motorSpeedB > 255) motorSpeedB = 255;
  else if (motorSpeedB < -255) motorSpeedB = -255;

  // Control Motor A
  if (motorSpeedA >= 0) {
    analogWrite(enA, motorSpeedA);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    analogWrite(enA, -motorSpeedA);
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }

  // Control Motor B
  if (motorSpeedB >= 0) {
    analogWrite(enB, motorSpeedB);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  } else {
    analogWrite(enB, -motorSpeedB);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }

  // Print the angles
  if (!isnan(angleXZ)) {
    Serial.print("AngleXZ: ");
    Serial.print(angleXZ);
  }
  if (!isnan(angleYZ)) {
    Serial.print("    AngleYZ: ");
    Serial.println(angleYZ);
  }

  delay(50);
}

void calibrateSensors() {
    const int numReadings = 1000;
    long gyroX_sum = 0, gyroY_sum = 0, gyroZ_sum = 0;
    long accX_sum = 0, accY_sum = 0, accZ_sum = 0;
    long magX_sum = 0, magY_sum = 0, magZ_sum = 0;

    int16_t magX, magY, magZ;

    Serial.println("Calibrating MPU9250. Please make sure the sensor is stationary and flat...");

    for (int i = 0; i < numReadings; i++) {
        mpu.getMotion9(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ, &magX, &magY, &magZ);
        gyroX_sum += gyroX;
        gyroY_sum += gyroY;
        gyroZ_sum += gyroZ;
        accX_sum += accX;
        accY_sum += accY;
        accZ_sum += accZ;
        magX_sum += magX;
        magY_sum += magY;
        magZ_sum += magZ;
        delay(10);
    }

    gyroX_offset = gyroX_sum / numReadings;
    gyroY_offset = gyroY_sum / numReadings;
    gyroZ_offset = gyroZ_sum / numReadings;
    accX_offset = accX_sum / numReadings;
    accY_offset = accY_sum / numReadings;
    accZ_offset = (accZ_sum / numReadings) - 16384; // Assuming 1g = 16384 at your sensor's scale
    magX_offset = magX_sum / numReadings;
    magY_offset = magY_sum / numReadings;
    magZ_offset = magZ_sum / numReadings;

    Serial.println("Finished Calibration");
    Serial.print("Gyro Offsets: ");
    Serial.print(gyroX_offset); Serial.print(", ");
    Serial.print(gyroY_offset); Serial.print(", ");
    Serial.print(gyroZ_offset);
    Serial.print("\nAccel Offsets: ");
    Serial.print(accX_offset); Serial.print(", ");
    Serial.print(accY_offset); Serial.print(", ");
    Serial.print(accZ_offset);
    Serial.print("\nMag Offsets: ");
    Serial.print(magX_offset); Serial.print(", ");
    Serial.print(magY_offset); Serial.print(", ");
    Serial.println(magZ_offset);
}

