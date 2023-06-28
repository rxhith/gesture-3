#define sensorPinA A0
#define sensorPinB A1
#define sensorPinC A2 // Flex Sensor is connected to this pin
#define PWMPin 6 // LED is attached to this Pin

float VCC = 5; // Arduino is powered with 5V VCC
float R2 = 10000; // 10K resistor is
float sensorMinResistance = 16700; // Value of the Sensor when it's flat
float sensorMaxResistance = 18200; // Value of the Sensor when it's bent at 90*

void setup() {
  Serial.begin(9600); // Initialize the serial with 9600 baud
  pinMode(sensorPinA, INPUT);
  pinMode(sensorPinB, INPUT);
  pinMode(sensorPinC, INPUT); // Sensor pin as input
}

void loop() {
  int ADCRawA = analogRead(sensorPinA);
  int ADCRawB = analogRead(sensorPinB);
  int ADCRawC = analogRead(sensorPinC);
  
  float ADCVoltageA = (ADCRawA * VCC) / 1023; // get the voltage e.g (512 * 5) / 1023 = 2.5V
  float ADCVoltageB = (ADCRawB * VCC) / 1023;
  float ADCVoltageC = (ADCRawC * VCC) / 1023;

  float ResistanceA = R2 * (VCC / ADCVoltageA - 1); // Calculate Resistance Value
  float ResistanceB = R2 * (VCC / ADCVoltageB - 1);
  float ResistanceC = R2 * (VCC / ADCVoltageC - 1);

  uint8_t ReadValueA = map(ResistanceA, sensorMinResistance, sensorMaxResistance, 0, 255); // map the values 16700 to 0  18200 to 255
  uint8_t ReadValueB = map(ResistanceB, sensorMinResistance, sensorMaxResistance, 0, 255);
  uint8_t ReadValueC = map(ResistanceC, sensorMinResistance, sensorMaxResistance, 0, 255);

  analogWrite(PWMPin, ReadValueA); // Generate PWM Signal for sensor A
analogWrite(PWMPin, ReadValueB); 
 analogWrite(PWMPin, ReadValueC); 

  // Print Debug Information
  
  Serial.print("  Read Value A: ");
  Serial.print(ReadValueA);
  Serial.print(" , ");
  Serial.print(ReadValueB);
  Serial.print(" ,");
 Serial.print(ReadValueC);
 Serial.println(" ");

  delay(1000);
}￼Enter
