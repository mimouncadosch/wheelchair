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
    int n;
    int revs = 0;
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
                  last_n = n;
                  revs += 1;
                  return 1;
              }
              else { 
                  return 0; 
              }
          }
//        return revs;
        }
};

class DirectionEncoder {
    // Class variables 
    int pinA;
    int pinB;
    int dir;  // Forward direction by default
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
            dir = 1;
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
          }
          last_n = n;  
          }  
        return dir;
        }
};

// Right wheel
SpeedEncoder se_right;
DirectionEncoder de_right;

void setup() {
    Serial.begin(9600);
    Serial.println("Setting up");
    se_right = SpeedEncoder(A0, 15);
    de_right = DirectionEncoder(A5, A4, 15);
    delay(1000);
    Serial.println("Ready");
}

int totRevs = 0;
int last_dir = 1;
int dir = 1;
int compensationRevs = 20;

int compensated = false;
void loop() {  
  
//    if (dir != last_dir && compensated == false) {
//      Serial.println("dir != last_dir");
//      totRevs -= compensationRevs;
//      compensated = true;
//    } 
//    dir = de_right.update();
//    totRevs += se_right.update() * de_right.update();
//    Serial.println(totRevs);
}
