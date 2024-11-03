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

// PID Tuning parameters
double Kp = 2, Ki = 0, Kd = 1.5; // Start with moderate values and adjust based on testing

// Create PID instances
PID yawPID(&yawInput, &yawOutput, &yawSetpoint, Kp, Ki, Kd, DIRECT);
PID pitchPID(&pitchInput, &pitchOutput, &pitchSetpoint, Kp, Ki, Kd, DIRECT);

// Smooth output change
double previousYawOutput = 0;
double outputChangeLimit = 3.0; // Adjust the limit for smoother changes (lower value for slower speed)

void setup() {
  Serial.begin(9600); // Initialize serial communication

  // Attach ESC pins
  yawESC.attach(24);
  pitchESC.attach(25);

  // Set initial setpoints
  yawSetpoint = 250;  // Desired yaw angle
  pitchSetpoint = 0 ;   // Neutral pitch (modify as needed)

  // Initialize PID controllers
  yawPID.SetMode(AUTOMATIC);
  pitchPID.SetMode(AUTOMATIC);

  // Set output limits for the PID controllers to match ESC control range
  yawPID.SetOutputLimits(-50, 50);  // Limiting PID output to slower speeds
  pitchPID.SetOutputLimits(-50, 50);  // Limiting PID output to slower speeds
    // Calibrate yaw ESC
  yawESC.writeMicroseconds(2000);  // Send max throttle
  delay(2000);  // Wait for ESC to recognize max throttle
  yawESC.writeMicroseconds(1000);  // Send min throttle
  delay(2000);  // Wait for ESC to recognize min throttle

  // Calibrate pitch ESC
  pitchESC.writeMicroseconds(2000);  // Send max throttle
  delay(2000);  // Wait for ESC to recognize max throttle
  pitchESC.writeMicroseconds(1000);  // Send min throttle
  delay(4000);  // Wait for ESC to recognize min throttle

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
  int yawSpeed = map(smoothYawOutput, -50, 50, 1000, 1500); // Narrower range for slower speeds
  int pitchSpeed = map(pitchOutput, -50, 50, 1000, 1500); // Narrower range for slower speeds

  // Send signals to ESCs
  yawESC.writeMicroseconds(yawSpeed);
  pitchESC.writeMicroseconds(pitchSpeed);

  // Print debug information
  Serial.print("Yaw: ");
  Serial.print(yawInput);
  Serial.print(" Pitch: ");
  Serial.println(pitchInput);

  delay(20); // Adjust the delay based on control speed requirements
}
