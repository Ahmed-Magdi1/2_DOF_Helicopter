double Pitch_current = 0;
double Yaw_current = 0;
const float sensitivity = 0.185; 
const int numSamples = 1000;
double zeroCurrentOffset1 = 0.0;
double zeroCurrentOffset2 = 0.0;

void setup() {
  Serial.begin(9600);

  double accumulatedOffset1 = 0.0;
  for (int i = 0; i < numSamples; i++) {
    accumulatedOffset1 += (0.00488281 * analogRead(A1)); // 5V / 1024 (Analog) = 0.00488281
    delay(1);
  }
  zeroCurrentOffset1 = accumulatedOffset1 / numSamples; 

  Serial.print("Calibrated Zero-Current Offset Voltage for Sensor A1 (V) = ");
  Serial.println(zeroCurrentOffset1, 4); 
  double accumulatedOffset2 = 0.0;
  for (int i = 0; i < numSamples; i++) {
    accumulatedOffset2 += (0.00488281 * analogRead(A2)); // 5V / 1024 (Analog) = 0.00488281
    delay(1);
  }
  zeroCurrentOffset2 = accumulatedOffset2 / numSamples; 

  Serial.print("Calibrated Zero-Current Offset Voltage for Sensor A2 (V) = ");
  Serial.println(zeroCurrentOffset2, 4); 

  delay(1000); 
}

void loop() {
  double accumulatedVoltage1 = 0;
  double accumulatedVoltage2 = 0;

  for (int i = 0; i < numSamples; i++) {
    accumulatedVoltage1 += (0.00488281 * analogRead(A1)); // 5V / 1024 (Analog) = 0.00488281
    delay(1);
  }

  for (int i = 0; i < numSamples; i++) {
    accumulatedVoltage2 += (0.00488281 * analogRead(A2)); // 5V / 1024 (Analog) = 0.00488281
    delay(1);
  }

  double avgVoltage1 = accumulatedVoltage1 / numSamples;
  double avgVoltage2 = accumulatedVoltage2 / numSamples;

  Pitch_current = (avgVoltage1 - zeroCurrentOffset1) / sensitivity;
  if (Pitch_current < 0) {
    Pitch_current = 0;
  }

  Yaw_current = (avgVoltage2 - zeroCurrentOffset2) / sensitivity;
  if (Yaw_current < 0) {
    Yaw_current = 0;
  }

  Serial.print("\nPitch_current = ");
  Serial.print(Pitch_current, 2); 

  Serial.print("\n Yaw_current = ");
  Serial.print(Yaw_current, 2); 
  
  delay(250);
}
