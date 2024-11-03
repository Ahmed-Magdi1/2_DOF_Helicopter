#include <Encoder.h>
#include <PID_v1.h>
#include <Servo.h>

// Define ticks per degree based on your encoder specifications (360 PPR)
#define ENCODER_TICKS_PER_DEGREE 1.0

// Encoder pins
Encoder yawEncoder(2, 3);  // Yaw encoder
Encoder pitchEncoder(18, 19);  // Pitch encoder

// ESC pins
Servo yawESC;
Servo pitchESC;

// Define Variables
double yawInput, yawOutput, yawSetpoint;
double pitchInput, pitchOutput, pitchSetpoint;

// PID Tuning parameters (adjust based on testing)
double Kp = 2, Ki = 0.0, Kd = 1.5;

// Create PID instances
PID yawPID(&yawInput, &yawOutput, &yawSetpoint, Kp, Ki, Kd, DIRECT);
PID pitchPID(&pitchInput, &pitchOutput, &pitchSetpoint, Kp, Ki, Kd, DIRECT);

// Smooth output change
double previousYawOutput = 0;
double outputChangeLimit = 3.0; // Adjust the limit for smoother changes (lower value for slower speed)

// Voltage sensor settings
const int voltagePin = A0; 
const float referenceVoltage = 5.0;  
const float voltageDividerRatio = 5.0;  
const float calibrationFactor = 1.03689567; 

// Current sensor settings
double Pitch_current = 0;
double Yaw_current = 0;
const float sensitivity = 0.100; 
const int numSamples = 200; // Reduced sampling count for faster performance
double zeroCurrentOffset1 = 0.0;
double zeroCurrentOffset2 = 0.0;
const float noiseThreshold = 0.06; // Define a small threshold for noise filtering

void setup() {
  Serial.begin(115200); // Increased baud rate for faster data transmission

  // Attach ESC pins
  yawESC.attach(24);
  pitchESC.attach(25);

  // Set initial setpoints
  yawSetpoint = 250;  // Desired yaw angle
  pitchSetpoint = 0;  // Neutral pitch (modify as needed)

  // Initialize PID controllers
  yawPID.SetMode(AUTOMATIC);
  pitchPID.SetMode(AUTOMATIC);

  // Set output limits for the PID controllers to match ESC control range
  yawPID.SetOutputLimits(-50, 50);  // Limiting PID output to slower speeds
  pitchPID.SetOutputLimits(-50, 50);  // Limiting PID output to slower speeds

  // ESC Calibration Process
  Serial.println("Calibrating ESCs...");

  // Calibrate yaw ESC
  yawESC.writeMicroseconds(2000);  // Send max throttle
  delay(2000);  // Wait for ESC to recognize max throttle
  yawESC.writeMicroseconds(1000);  // Send min throttle
  delay(2000);  // Wait for ESC to recognize min throttle

  // Calibrate pitch ESC
  pitchESC.writeMicroseconds(2000);  // Send max throttle
  delay(2000);  // Wait for ESC to recognize max throttle
  pitchESC.writeMicroseconds(1000);  // Send min throttle
  delay(2000);  // Wait for ESC to recognize min throttle

  Serial.println("ESC Calibration Complete");

  // Current Sensor Calibration
  double accumulatedOffset1 = 0.0;
  for (int i = 0; i < numSamples; i++) {
    accumulatedOffset1 += (0.00488281 * analogRead(A7)); // 5V / 1024 (Analog) = 0.00488281
  }
  zeroCurrentOffset1 = accumulatedOffset1 / numSamples; 

  Serial.print("Calibrated Zero-Current Offset Voltage for Sensor A1 (V) = ");
  Serial.println(zeroCurrentOffset1, 4); 

  double accumulatedOffset2 = 0.0;
  for (int i = 0; i < numSamples; i++) {
    accumulatedOffset2 += (0.00488281 * analogRead(A2)); // 5V / 1024 (Analog) = 0.00488281
  }
  zeroCurrentOffset2 = accumulatedOffset2 / numSamples; 

  Serial.print("Calibrated Zero-Current Offset Voltage for Sensor A2 (V) = ");
  Serial.println(zeroCurrentOffset2, 4); 

  delay(1000); 
}

void loop() {
  // Read encoder values and convert to degrees
  yawInput = yawEncoder.read() / ENCODER_TICKS_PER_DEGREE;
  pitchInput = pitchEncoder.read() / ENCODER_TICKS_PER_DEGREE;

  // Compute PID outputs
  yawPID.Compute();
  pitchPID.Compute();

  // Smooth the PID output change to prevent abrupt movements
  double smoothYawOutput = previousYawOutput + constrain(yawOutput - previousYawOutput, -outputChangeLimit, outputChangeLimit);
  previousYawOutput = smoothYawOutput;

  // Map PID output to ESC control range with a slower speed limit
  int yawSpeed = map(smoothYawOutput, -50, 50, 1100, 1500); // Adjusted range for steadier speeds
  int pitchSpeed = map(pitchOutput, -50, 50, 1100, 1500); // Adjusted range for steadier speeds

  // Send signals to ESCs
  yawESC.writeMicroseconds(yawSpeed);
  pitchESC.writeMicroseconds(pitchSpeed);

  // Read voltage value from the sensor and calculate voltage
  int sensorValue = analogRead(voltagePin);  
  float voltage = (sensorValue / 1024.0) * referenceVoltage * voltageDividerRatio * calibrationFactor;
  
  // Read current values from sensors
  double accumulatedVoltage1 = 0;
  double accumulatedVoltage2 = 0;

  for (int i = 0; i < numSamples; i++) {
    accumulatedVoltage1 += (0.00488281 * analogRead(A7)); // 5V / 1024 (Analog) = 0.00488281
  }

  for (int i = 0; i < numSamples; i++) {
    accumulatedVoltage2 += (0.00488281 * analogRead(A2)); // 5V / 1024 (Analog) = 0.00488281
  }

  double avgVoltage1 = accumulatedVoltage1 / numSamples;
  double avgVoltage2 = accumulatedVoltage2 / numSamples;

  // Calculate current with noise threshold filtering
  Pitch_current = (avgVoltage1 - zeroCurrentOffset1) / sensitivity;
  if (abs(Pitch_current) < noiseThreshold) {
    Pitch_current = 0.0; // Filter out small noise values
  }

  Yaw_current = (avgVoltage2 - zeroCurrentOffset2) / sensitivity;
  if (abs(Yaw_current) < noiseThreshold) {
    Yaw_current = 0.0; // Filter out small noise values
  }

  // Print all data in one line without converting to String
  Serial.print("Voltage: ");
  Serial.print(voltage);
  Serial.print(" V | Yaw: ");
  Serial.print(yawInput);
  Serial.print(" | Pitch: ");
  Serial.print(pitchInput);
  Serial.print(" | Pitch_current: ");
  Serial.print(Pitch_current, 2);
  Serial.print(" A | Yaw_current: ");
  Serial.print(Yaw_current, 2);
  Serial.println(" A");

  delay(50); // Adjust delay as needed for speed
}
