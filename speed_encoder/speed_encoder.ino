/**
M. Delmar, 7/2015
This program counts the number of revolutions in each wheel.
It is used to compute the wheelchair position with accuracy.
*/

unsigned long start, finished, elapsed;

//  Read voltage off Sparkfun photointerrupter
int pin = A0;
int ticksRight  = 0;
int n1;
int last_n1 = LOW;
int threshold = 3.5;

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
//  start = micros();
  n1 = readSensor(pin);
  
  if (n1 != last_n1) {
    // New tick detected
    ticksRight += 1;
    Serial.print("n: ");
    Serial.println(ticksRight);
    last_n1 = n1;
  }
//  finished=micros();
//  elapsed=finished-start;
//  Serial.print("elapsed ");
//  Serial.println(elapsed);
//  delay(0.1);
}

