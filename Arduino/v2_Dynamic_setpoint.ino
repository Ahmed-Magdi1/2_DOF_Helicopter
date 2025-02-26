#include <Encoder.h>
#include <PID_v1.h>
#include <Servo.h>

// Define encoder resolution based on your encoder specifications (PPR)
#define ENCODER_TICKS_PER_DEGREE 1.0

// Define encoder pins
Encoder yawEncoder(20, 21);
Encoder pitchEncoder(2, 3);

// ESC pins
Servo yawESC;
Servo pitchESC;

// Define Variables
double yawInput, yawOutput, yawSetpoint;
double pitchInput, pitchOutput, pitchSetpoint;

// PID Tuning parameters
double Kp = 2, Ki = 0.0, Kd = 1.5;
PID yawPID(&yawInput, &yawOutput, &yawSetpoint, Kp, Ki, Kd, DIRECT);
PID pitchPID(&pitchInput, &pitchOutput, &pitchSetpoint, Kp, Ki, Kd, DIRECT);

// Smooth output change
double previousYawOutput = 0;
double outputChangeLimit = 3.0;

// Voltage sensor settings
const int voltagePin = A0; 
const float referenceVoltage = 5.0;  
const float voltageDividerRatio = 5.0;  
const float calibrationFactor = 1.03689567; 

// Current sensor settings
double Pitch_current = 0, Yaw_current = 0;
const float sensitivity = 0.100; 
const int numSamples = 50;
double zeroCurrentOffset1 = 0.0, zeroCurrentOffset2 = 0.0;
const float noiseThreshold = 0.5;

// Sampling time control
unsigned long prev_time = 0;
const int sample_time = 10;

// Dynamic setpoint control
const int changeInterval = 4000; // Change every 5 seconds
unsigned long lastChangeTime = 0;
int pitchSetpoints[] = {240, 245, 250, 255, 260, 265, 270, 275, 270, 265, 260, 255, 250, 245};
int yawSetpoints[] = {0, 5, 10, 15, 10, 5, 0}; //{0, 5, 10, 15, 10, 5, 0};
int pitchIndex = 0;
int yawIndex = 0;

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("***********************************************");
  Serial.println("                                               ");
  Serial.println("    🚁 Welcome to the Raptor2 Helicopter! 🚁   ");
  Serial.println("                                               ");
  Serial.println("***********************************************");
  Serial.println();
  delay(3000);

  yawESC.attach(33, 1000, 2000);  // Pin 33, range from 1000 to 2000 microseconds
  pitchESC.attach(32, 1000, 2000);  // Pin 32, range from 1000 to 2000 microseconds

  // Initialize ESCs by sending max throttle (2000)
  Serial.println("Starting ESC Calibration...");
  
  yawESC.writeMicroseconds(2000);  // Max throttle
  pitchESC.writeMicroseconds(2000);  // Max throttle
  delay(4000);  // Wait for 3 seconds to allow ESCs to detect max throttle

  // Now, set the ESCs to min throttle (1000)
  yawESC.writeMicroseconds(1000);  // Min throttle
  pitchESC.writeMicroseconds(1000);  // Min throttle
  delay(4000);  // Wait for 3 seconds to allow ESCs to detect min throttle

  // Finish calibration by stopping the motors
  yawESC.writeMicroseconds(1000);  // Stop motor
  pitchESC.writeMicroseconds(1000);  // Stop motor
  
  Serial.println("ESC Calibration Completed!");

  yawSetpoint = yawSetpoints[yawIndex];
  pitchSetpoint = pitchSetpoints[pitchIndex];

  yawPID.SetMode(AUTOMATIC);
  pitchPID.SetMode(AUTOMATIC);

  yawPID.SetOutputLimits(-50, 50);
  pitchPID.SetOutputLimits(-50, 50);

  for (int i = 0; i < numSamples; i++) {
    zeroCurrentOffset1 += (0.00488281 * analogRead(A6));
    zeroCurrentOffset2 += (0.00488281 * analogRead(A2));
  }
  zeroCurrentOffset1 /= numSamples;
  zeroCurrentOffset2 /= numSamples;

  Serial.print("Calibrated Zero-Current Offset A1 (V) = ");
  Serial.println(zeroCurrentOffset1, 4);
  Serial.print("Calibrated Zero-Current Offset A2 (V) = ");
  Serial.println(zeroCurrentOffset2, 4);
  Serial.println("-----------------------------------------------");
  Serial.print("Voltage"); Serial.print(",");
  Serial.print("Pitch_Angle"); Serial.print(",");
  Serial.print("Yaw_Angle"); Serial.print(",");
  Serial.print("Pitch_Current"); Serial.print(",");
  Serial.print("Yaw_Current"); Serial.print(",");
  Serial.print("Yaw_PWM"); Serial.print(",");
  Serial.print("Pitch_PWM"); 
  delay(3000);
}

void loop() {
  if (millis() - prev_time >= sample_time) {
    prev_time = millis();
    
    // Update setpoints every 5 seconds
    if (millis() - lastChangeTime > changeInterval) {
      lastChangeTime = millis();
      pitchIndex = (pitchIndex + 1) % 7;
      yawIndex = (yawIndex + 1) % 7;
      pitchSetpoint = pitchSetpoints[pitchIndex];
      yawSetpoint = yawSetpoints[yawIndex];
    }

    yawInput = yawEncoder.read() / ENCODER_TICKS_PER_DEGREE;
    pitchInput = pitchEncoder.read() / ENCODER_TICKS_PER_DEGREE;

    yawPID.Compute();
    pitchPID.Compute();

    double smoothYawOutput = previousYawOutput + constrain(yawOutput - previousYawOutput, -outputChangeLimit, outputChangeLimit);
    previousYawOutput = smoothYawOutput;

    int yawSpeed = map(smoothYawOutput, -50, 50, 1050, 1500);
    int pitchSpeed = map(pitchOutput, -50, 50, 1050, 1500);

    yawESC.writeMicroseconds(yawSpeed);
    pitchESC.writeMicroseconds(pitchSpeed);

    float voltage = (analogRead(voltagePin) / 1024.0) * referenceVoltage * voltageDividerRatio * calibrationFactor;
    
    double avgVoltage1 = 0, avgVoltage2 = 0;
    for (int i = 0; i < numSamples; i++) {
      avgVoltage1 += (0.00488281 * analogRead(A6));
      avgVoltage2 += (0.00488281 * analogRead(A2));
    }
    avgVoltage1 /= numSamples;
    avgVoltage2 /= numSamples;

    Pitch_current = (avgVoltage1 - zeroCurrentOffset1) / sensitivity;
    Yaw_current = (avgVoltage2 - zeroCurrentOffset2) / sensitivity;
    if (abs(Pitch_current) < noiseThreshold) Pitch_current = 0.0;
    if (abs(Yaw_current) < noiseThreshold) Yaw_current = 0.0;

    Serial.print(voltage); Serial.print(",");
    Serial.print(pitchInput * 0.25); Serial.print(",");
    Serial.print(yawInput * 0.25); Serial.print(",");
    Serial.print(Pitch_current, 2); Serial.print(",");
    Serial.print(Yaw_current, 2); Serial.print(",");
    Serial.print(yawSpeed); Serial.print(",");
    Serial.println(pitchSpeed);
  }
}
