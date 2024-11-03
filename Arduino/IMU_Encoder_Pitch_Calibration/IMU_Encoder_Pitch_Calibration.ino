#include <Encoder.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// Define ticks per degree based on your encoder specifications (360 PPR)
#define ENCODER_TICKS_PER_DEGREE 1.0

// Encoder pins
Encoder yawEncoder(18, 19);  // Yaw encoder
Encoder pitchEncoder(2, 3);  // Pitch encoder

// BNO055 IMU
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Define Variables
double yawInput, pitchInput;
double bnoRoll = 0.0;

void setup() {
  Serial.begin(115200); // Set up serial communication at 115200 baud rate

  // Initialize BNO055
  if (!bno.begin()) {
    Serial.print("BNO055 not detected. Check wiring or I2C address.");
    while (1);
  }
  bno.setExtCrystalUse(true);
  
  // Delay for sensor initialization
  delay(1000);
}

void loop() {
  // Read encoder values and convert to degrees
  yawInput = yawEncoder.read() / ENCODER_TICKS_PER_DEGREE;
  pitchInput = pitchEncoder.read() / ENCODER_TICKS_PER_DEGREE;

  // Get IMU data
  sensors_event_t event;
  bno.getEvent(&event);

  // Get the roll angle from BNO055
  bnoRoll = event.orientation.y; // Assuming 'y' axis holds the roll value

  Serial.print("Pitch angle (Encoder): ");
  Serial.print(pitchInput * 0.25); // Scale if needed
  Serial.print("° | Pitch angle (BNO055): ");
  Serial.print(bnoRoll);
  Serial.println("°");

  // If BNO055 roll is near zero, print corresponding encoder value
  if (abs(bnoRoll) < 0.5) { // Consider zero within a small tolerance range
    Serial.print("BNO055 Pitch is zero. Corresponding Encoder Pitch value: ");
    Serial.println(pitchInput); // Assuming pitch encoder is related to roll in your setup
  }

  delay(20); // Adjust delay as needed for reading speed
}
