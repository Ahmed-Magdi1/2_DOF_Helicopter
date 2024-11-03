#include <Encoder.h>
#include <PID_v1.h>
#include <Servo.h>

// Define ticks per degree based on your encoder specifications (360 PPR)
#define ENCODER_TICKS_PER_DEGREE 1.0

// Encoder pins
Encoder yawEncoder(18, 19);  // Yaw encoder
Encoder pitchEncoder(2, 3);  // Pitch encoder

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
const float noiseThreshold = 0.5; // Define a small threshold for noise filtering

void setup() {
  Serial.begin(115200); // Increased baud rate for faster data transmission

  // Attach ESC pins and set the pulse width range
  yawESC.attach(25, 1000, 2000);
  pitchESC.attach(24, 1000, 2000);

  // Send the minimum throttle signal to exit calibration or programming mode
  yawESC.writeMicroseconds(1000);
  pitchESC.writeMicroseconds(1000);

  // Allow some time for the ESCs to initialize
  delay(5000);

  // Set initial setpoints
  yawSetpoint = 0;  // Desired yaw angle
  pitchSetpoint = 237;  // Neutral pitch (modify as needed)

  // Initialize PID controllers
  yawPID.SetMode(AUTOMATIC);
  pitchPID.SetMode(AUTOMATIC);

  // Set output limits for the PID controllers to match ESC control range
  yawPID.SetOutputLimits(-50, 50);  // Limiting PID output to slower speeds
  pitchPID.SetOutputLimits(-50, 50);  // Limiting PID output to slower speeds

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
  int yawSpeed = map(smoothYawOutput, -50, 50, 1030, 1350); // Adjusted range for steadier speeds
  int pitchSpeed = map(pitchOutput, -50, 50, 1030, 1200); // Adjusted range for steadier speeds

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

  // Print all relevant data, including ESC signals
  Serial.print("Voltage : ");
  Serial.print(voltage);
  Serial.print(" V | Pitch angle : ");
  Serial.print(pitchInput * 0.25);
  Serial.print("° | Yaw angle : ");
  Serial.print(yawInput * 0.25);
  Serial.print("° | Pitch_current : ");
  Serial.print(Pitch_current, 2);
  Serial.print(" A | Yaw_current : ");
  Serial.print(Yaw_current, 2);
  Serial.print(" A | Yaw ESC Signal: ");
  Serial.print(yawSpeed);
  Serial.print(" µs | Pitch ESC Signal: ");
  Serial.print(pitchSpeed);
  Serial.println(" µs");

  delay(20); // Adjust delay as needed for speed
}
