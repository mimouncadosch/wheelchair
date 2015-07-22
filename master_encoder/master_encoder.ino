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
    int revs = 0;
    int n;
    int last_n;
    
    // Constructor
    public:
        SpeedEncoder(int myPin) {
            pin = myPin;
            last_n = LOW;
        }
        
        int update() {
              // Update sensor value
              n = readSensor(pin);    
              
              if (n != last_n) {
                  revs += 1;
                  Serial.println(revs);
                  last_n = n;
              }
         }     
};

class DirectionEncoder {
    // Class variables 
    int pinA;
    int pinB;
    int dir;  // Forward direction by default
    int n;
    int last_n;
    
    // Constructor
    public:
        DirectionEncoder(int myPinA, int myPinB) {
            pinA = myPinA;
            pinB = myPinB;
            last_n = LOW;
            dir = 1;
        }
        
        int update() {
              // Update sensor value
              n = readSensor(pinB);  
              if ( (last_n == LOW) && (n == HIGH) ) {
                if ( readSensor(pinA) == LOW )  {
                    dir = 1;
                    Serial.println(dir);
                }
                else {
                    dir = -1;
                    Serial.println(dir);
                }
          }
          last_n = n;
          return dir;
        }
};

// Right wheel
SpeedEncoder se_right = SpeedEncoder(A0);
DirectionEncoder de_right = DirectionEncoder(A4,A5);


void setup() {
    Serial.begin(9600);
    Serial.println("Setting up");
    delay(1000);
    Serial.println("Ready");
}

void loop() {
    se_right.update();
    de_right.update();
}
