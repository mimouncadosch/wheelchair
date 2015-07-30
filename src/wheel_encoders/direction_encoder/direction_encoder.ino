// Right wheel settings
int analogPinA = A2;
int analogPinB = A1;
int revsRight  = 0;
String r;

// Left wheel settings
int analogPinC = A4;
int analogPinD = A3;
int revsLeft   = 0;
String l;

// Above threshold: 1, Below threshold: 0
int threshold = 3.0;

int n1;
int last_n1 = LOW;

int n2;
int last_n2 = LOW;

void setup() {
  Serial.begin(9600);
}

// Read optical IR sensor, convert analog signal to digital value (0 or 1)
int readSensor(int pin) {
  // read the input on analog pin 0:
  int sensorVal = analogRead(pin);  // Read sensor value from LED
  float voltage = sensorVal * (5.0 / 1023.0);  // Voltage
  return (voltage >= threshold) ? HIGH : LOW;
}

void loop() {
  
  n1 = readSensor(analogPinB);
  n2 = readSensor(analogPinD);
//  Serial.println(n1);
  // Right wheel revolutions  
  if ((last_n1 == LOW) && (n1 == HIGH)) {
    if (readSensor(analogPinA) == LOW) { //analogPinB
      revsLeft--;
    } else {
      revsLeft++;
    }
    Serial.print("left: ");
    Serial.println(revsLeft);
  } 
  last_n1 = n1;
  
//  // Left wheel revolutions
//  if ((last_n2 == LOW) && (n2 == HIGH)) {
//    if (readSensor(analogPinC) == LOW) {
//      revsRight--;
//    } else {
//      revsRight++;
//    }
////    Serial.print("right: ");
////    Serial.println(revsRight);
//  }
//  last_n2 = n2;
  
//  Serial.print("right: ");
//  Serial.print(revsRight);
//  Serial.print("\t left: ");
//  Serial.println(revsLeft);
 
}
