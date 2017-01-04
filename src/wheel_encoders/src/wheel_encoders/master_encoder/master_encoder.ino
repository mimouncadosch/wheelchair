/**
This program combines readings from the speed and direction encoders to provide the number and direction of wheel "ticks".
**/

// Read optical IR sensor, convert analog signal to digital value (0 or 1)
int readSensor(int pin, float threshold) {
  // Above threshold: HIGH, Below threshold: LOW
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
            n = readSensor(pin, 3.0);    
            if (n != last_n) {
                revs += 1;
                last_n = n;
//                show && Serial.println(revs);
                return 1;
            }
            else { return 0; }
        }
};

class DirectionEncoder {
    // Class variables 
    private:
        int pinA;
        int pinB;
        int n;
        int last_n;
        
    // Constructor
    public:  
        int dir;
        int last_dir;
        DirectionEncoder(int myPinA, int myPinB) {            
            pinA = myPinA;
            pinB = myPinB;
            last_n = LOW;
            dir = 1;
            last_dir = 1;
        }
        
        int update(bool show = false) {
              n = readSensor(pinB, 3.0);
              if ( (last_n == LOW) && (n == HIGH) ) {
                if ( readSensor(pinA, 3.0) == LOW )  {
                    dir = 1;
                    last_n = n;
                    return dir;
                }
                else {
                    dir = -1;
                    last_n = n;
                    return dir;
                }
          }
//          last_n = n;
//          return dir;
        }
};


SpeedEncoder se_right = SpeedEncoder(A0);
SpeedEncoder se_left  = SpeedEncoder(A1);
DirectionEncoder de_right = DirectionEncoder(A1, A2);
DirectionEncoder de_left = DirectionEncoder(A3, A2);

void setup() {
    Serial.begin(9600);
}

int ticksRight = 0;
int ticksLeft = 0;

int compensationTicks = 20;

// This boolean variable indicates whether the speed encoder has been compensated for the delay in the direction encoder.
int compensatedRight = false;
int compensatedLeft = false;

void loop() {
  
      de_right.update();
      de_left.update();
      
      // Check if change in direction right wheel
      if (de_right.dir != de_right.last_dir && compensatedRight == false) {
//          Serial.println("Change in direction");
          de_right.last_dir = de_right.dir;
          ticksRight -= compensationTicks;
          compensatedRight = true;

      }
      
      // Check if change in direction left wheel
      if (de_left.dir != de_left.last_dir && compensatedLeft == false) {
//          Serial.println("Change in direction");
          de_left.last_dir = de_left.dir;
          ticksLeft -= compensationTicks;
          compensatedLeft = true;        
      }
      ticksRight += se_right.update() * de_right.dir;
      ticksLeft += se_left.update() * de_left.dir;
      Serial.print("right: ");
      Serial.print(ticksRight);
      Serial.print("\t");
      Serial.print("left: ");
      Serial.println(ticksLeft);
      
      compensatedRight = false;
      compensatedLeft = false;
}
