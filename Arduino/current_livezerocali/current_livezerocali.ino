double Voltage = 0;
double Current = 0;
const float sensitivity = 0.100; // Adjust based on your sensor model (e.g., 0.185 for ACS712-5A)
const int numSamples = 200;
double zeroCurrentOffset = 0.0;

void setup() {
  Serial.begin(9600);

  // Calibration Phase: Measure zero-current offset with no load connected
  double accumulatedOffset = 0.0;
  for (int i = 0; i < numSamples; i++) {
    accumulatedOffset += (0.00488281 * analogRead(A7)); // 5V / 1024 (Analog) = 0.00488281
    delay(1);
  }
  zeroCurrentOffset = accumulatedOffset / numSamples; // Calculate average zero-current voltage

  Serial.print("Calibrated Zero-Current Offset Voltage (V) = ");
  Serial.println(zeroCurrentOffset, 4); // Print the calibrated offset voltage with 4 decimal places

  delay(1000); // Short delay to allow user to see the calibration result
}

void loop() {
  Voltage = 0; // Reset voltage accumulation

  // Voltage is Sensed 1000 Times for precision
  for (int i = 0; i < numSamples; i++) {
    Voltage += (0.00488281 * analogRead(A7)); // 5V / 1024 (Analog) = 0.00488281
    delay(1);
  }

  // Calculate average voltage
  Voltage = Voltage / numSamples;

  // Subtract the offset and calculate current
  Current = (Voltage - zeroCurrentOffset) / sensitivity; 

  // Ensure small negative values due to noise are set to zero
  if (Current < 0) {
    Current = 0;
  }

  Serial.print("\nVoltage Sensed (V) = ");
  Serial.print(Voltage, 2); // Display voltage with 2 decimal places
  Serial.print("\t Current (A) = ");
  Serial.print(Current, 2); // Display current with 2 decimal places

  delay(250);
}
