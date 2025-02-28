#include <Encoder.h>
#include <PID_v1.h>
#include <Servo.h>

// Define encoder resolution based on your encoder specifications (PPR)
#define ENCODER_TICKS_PER_DEGREE 1.0

// Encoder pins
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

// PRBS settings
const int changeInterval = 3000;  // Change every 4 seconds
unsigned long lastChangeTime = 0;
int prbsState = 1;  // Initial state

// Function to generate PRBS sequence
int generatePRBS() {
  prbsState = ((prbsState >> 1) ^ (-(prbsState & 1) & 0xB400)) & 0xFFFF;
  return prbsState & 1;  // Return only the LSB (binary output)
}

void setup() {
  Serial.begin(115200);
  Serial.println("***********************************************");
  Serial.println("    🚁 Welcome to the PRBS-Enabled Helicopter! 🚁   ");
  Serial.println("***********************************************");
  delay(3000);

  yawESC.attach(33, 1000, 2000);
  pitchESC.attach(32, 1000, 2000);

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


  yawSetpoint = generatePRBS() ? 10 : 0;  // Initial PRBS-based yaw
  pitchSetpoint = generatePRBS() ? 280 : 200;  // Initial PRBS-based pitch

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
}

void loop() {
  if (millis() - prev_time >= sample_time) {
    prev_time = millis();
    
    // Update PRBS-based setpoints every 4 seconds
    if (millis() - lastChangeTime > changeInterval) {
      lastChangeTime = millis();
      yawSetpoint = generatePRBS() ? 10 : 0;
      pitchSetpoint = generatePRBS() ? 280 : 200;
    }

    yawInput = yawEncoder.read() / ENCODER_TICKS_PER_DEGREE;
    pitchInput = pitchEncoder.read() / ENCODER_TICKS_PER_DEGREE;

    yawPID.Compute();
    pitchPID.Compute();

    double smoothYawOutput = previousYawOutput + constrain(yawOutput - previousYawOutput, -outputChangeLimit, outputChangeLimit);
    previousYawOutput = smoothYawOutput;

    int yawSpeed = map(smoothYawOutput, -50, 50, 1050,1400 );
    int pitchSpeed = map(pitchOutput, -50, 50, 1050, 1600);

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
