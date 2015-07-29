//unsigned long start, finished, elapsed;

// Above threshold: HIGH, Below threshold: LOW
int threshold = 4;

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
    
    // Constructor
    public:        
        SpeedEncoder(int myPin) {
            pin = myPin;
            last_n = LOW;
        }
        
        int update(bool show = false) {
            // Update sensor value
            n = readSensor(pin);    
            if (n != last_n) {
                revs += 1;
                last_n = n;
                return 1;
//                show && Serial.println(revs);
            }
            else { return 0; }
        }
};

class DirectionEncoder {
    // Class variables 
    int pinA;
    int pinB;
    int n;
    int last_n;
    int dir;
    int last_dir;
    
    // Constructor
    public:        
        DirectionEncoder(int myPinA, int myPinB) {
            pinA = myPinA;
            pinB = myPinB;
            last_n = LOW;
            dir = 1;
            last_dir = 1;
        }
        
        int update(bool show = false) {
              // Update sensor value
              n = readSensor(pinB);  
              if ( (last_n == LOW) && (n == HIGH) ) {
                if ( readSensor(pinA) == LOW )  {
                    setDir(1);
                    show && Serial.println(dir);
                }
                else {
                    setDir(-1);
                    show && Serial.println(dir);
                }
          }
          last_n = n;
          return dir;
        }
        
        void setDir(int newDir) {
            dir = newDir; 
        }
        void setLastDir(int newDir) { 
          last_dir = newDir;
        }

        int getDir() {
          return dir;
        }
};

SpeedEncoder se_right = SpeedEncoder(A0);
SpeedEncoder se_left  = SpeedEncoder(A1);
DirectionEncoder de_right = DirectionEncoder(A4, A5);
DirectionEncoder de_left = DirectionEncoder(A3, A2);

void setup() {
    Serial.begin(9600);
}

int revsRight = 0;
int compensationRevs = 20;
int compensated = false;

void loop() {  
//      start = micros();
//      revsLeft += se_left.update();
//      de_right.update();
//      Serial.println(de_right.getDir());
//      de_left.update();
      
//      finished=micros();
//      elapsed=finished-start;
//      Serial.print(elapsed);
//      Serial.println(" microseconds elapsed");
      
//    if (de_right.dir != de_right.last_dir && compensated == false) {
////      Serial.println("dir != last_dir");
//        revsRight -= compensationRevs;
//        compensated = true;
//        de_right.last_dir = de_right.dir;
//    } 
    revsRight += se_right.update();
//    revsRight += se_right.update() * de_right.dir;
    Serial.println(revsRight);
}
