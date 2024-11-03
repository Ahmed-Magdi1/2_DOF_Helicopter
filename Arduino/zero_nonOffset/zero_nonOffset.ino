double Voltage = 0;
const int numSamples = 1000;

void setup() {
  Serial.begin(9600);
}

void loop() {
  Voltage = 0; // Reset voltage accumulation

  // Measure voltage 1000 times to find the zero-current offset
  for (int i = 0; i < numSamples; i++) {
    Voltage += (0.00488281 * analogRead(A0)); // 5V / 1024 (Analog) = 0.00488281
    delay(1);
  }

  // Calculate average zero-current voltage
  Voltage = Voltage / numSamples;

  // Print the measured zero-current offset voltage
  Serial.print("Zero-Current Offset Voltage (V) = ");
  Serial.println(Voltage, 4); // Display voltage with 4 decimal places for precision

  delay(2000);
}
