volatile unsigned int temp1, Pitch_encoder = 0;  // Pitch encoder counters
volatile unsigned int temp2, Yaw_encoder = 0;  // Yaw encoder counters

void setup() {
  Serial.begin(9600);

  pinMode(2, INPUT_PULLUP);  
  pinMode(3, INPUT_PULLUP);  

  pinMode(18, INPUT_PULLUP);  
  pinMode(19, INPUT_PULLUP);  

  attachInterrupt(digitalPinToInterrupt(2), encoder1_A_ISR, RISING);  // Pin 2 rising pulse for encoder 1
  attachInterrupt(digitalPinToInterrupt(3), encoder1_B_ISR, RISING);  // Pin 3 rising pulse for encoder 1

  attachInterrupt(digitalPinToInterrupt(18), encoder2_A_ISR, RISING);  // Pin 18 rising pulse for encoder 2
  attachInterrupt(digitalPinToInterrupt(19), encoder2_B_ISR, RISING);  // Pin 19 rising pulse for encoder 2
}

void loop() {
  if (Pitch_encoder != temp1 || Yaw_encoder != temp2) {
    Serial.print("Pitch encoder: ");
    Serial.print(Pitch_encoder);
    Serial.print(" | Yaw encoder 2: ");
    Serial.println(Yaw_encoder);
    temp1 = Pitch_encoder;
    temp2 = Yaw_encoder;
  }
}

void encoder1_A_ISR() {
  if (digitalRead(3) == LOW) {  
    Pitch_encoder++;
  } else {
    Pitch_encoder--;
  }
}

void encoder1_B_ISR() {
  if (digitalRead(2) == LOW) {  
    Pitch_encoder--;
  } else {
    Pitch_encoder++;
  }
}

void encoder2_A_ISR() {
  if (digitalRead(19) == LOW) {  
    Yaw_encoder++;
  } else {
    Yaw_encoder--;
  }
}

void encoder2_B_ISR() {
  if (digitalRead(18) == LOW) { 
    Yaw_encoder--;
  } else {
    Yaw_encoder++;
  }
}
