const int voltagePin = A0; 
const float referenceVoltage = 5.0;  
const float voltageDividerRatio = 5.0;  
const float calibrationFactor = 1.03689567; 

void setup() {
  Serial.begin(9600); 
}

void loop() {
  int sensorValue = analogRead(voltagePin);  
  float voltage = (sensorValue / 1024.0) * referenceVoltage * voltageDividerRatio * calibrationFactor;
  
  Serial.print("Voltage: ");
  Serial.print(voltage);
  Serial.println(" V");

  // delay(1000);  
}
