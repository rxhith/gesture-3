

// Flex sensor analog pins
const int flexPin1 = A0;
const int flexPin2 = A5;
const int flexPin3 = A4;



void setup() {
  Serial.begin(115200);
}

void loop() {
  // Read flex sensor values
  int flexValue1 = analogRead(flexPin1);
  int flexValue2 = analogRead(flexPin2);
  int flexValue3 = analogRead(flexPin3);

  // Send sensor data to NodeMCU via serial connection
  Serial.print("FLEX:");
  Serial.print(flexValue1);
  Serial.print(",");
  Serial.print(flexValue2);
  Serial.print(",");
  Serial.println(flexValue3);
  delay(100); // Adjust delay as per your requirements
}￼Enter
