

// Above threshold: HIGH, Below threshold: LOW
int threshold = 2.5;

// Read optical IR sensor, convert analog signal to digital value (0 or 1)
int readSensor(int pin) {
  float voltage = analogRead(pin) * (5.0 / 1023.0);  // Voltage
  return (voltage >= threshold) ? HIGH : LOW;
}

class SpeedEncoder  {
    // Class variables
    int pin;
    int revs;
    int n;
    int last_n;
    int updateInterval;      // interval between updates
    unsigned long lastUpdate; // last update of position
    
    // Constructor
    public:
        SpeedEncoder(int myPin, int interval) {
            pin = myPin;
            last_n = LOW;
            updateInterval = interval;
        }
        
        int update() {
          if((micros() - lastUpdate) > updateInterval) { // time to update
              lastUpdate = micros();
              
              // Update sensor value
              n = readSensor(pin);    
              if (n != last_n) {
                  revs += 1;
//                  Serial.println(revs);
                  last_n = n;
              }
          }
        }
};

class DirectionEncoder {
    // Class variables 
    int pinA;
    int pinB;
    int dir;
    int n;
    int last_n;
    int updateInterval;
    unsigned long lastUpdate;
    
    // Constructor
    public:
        DirectionEncoder(int myPinA, int myPinB, int interval) {
            pinA = myPinA;
            pinB = myPinB;
            last_n = LOW;
            updateInterval = interval;
        }
        
        int update() {
          if((micros() - lastUpdate) > updateInterval) { // time to update
              lastUpdate = micros();
              
              // Update sensor value
              n = readSensor(pinB);  
              if ( (last_n == LOW) && (n == HIGH) ) {
                if ( readSensor(pinA) == LOW )  {
                    dir = -1;
                }
                else {
                    dir = 1;
                }
//              Serial.println(dir);
          }
          last_n = n;  
          }  
        }
};

SpeedEncoder se = SpeedEncoder(A3, 100);
DirectionEncoder de = DirectionEncoder(A0, A1, 100);
int revs;
int dir;
void setup() {
    Serial.begin(9600);
}

void loop() {
    revs = se.update();
//    dir = de.update();
    Serial.println(revs);
}
