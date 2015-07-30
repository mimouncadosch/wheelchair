/**
This program combines readings from the speed and direction encoders to provide the number and direction of wheel "ticks".
**/

// Global variables
int pinA = A0;
int ticksRight = 0;
int pinB = A2;
int pinC = A1;
int dir;
int last_dir = 1;

int n;
int last_n = LOW;


int compensationTicks = 20;
int compensatedRight = false;  // This boolean variable indicates whether the speed encoder has been compensated for the delay in the direction encoder.


// Read optical IR sensor, convert analog signal to digital value (0 or 1)
int readSensor(int pin, float threshold) {
  // Above threshold: HIGH, Below threshold: LOW
  float voltage = analogRead(pin) * (5.0 / 1023.0);  // Voltage
  return (voltage >= threshold) ? HIGH : LOW;
}

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
                }
                else {
                    dir = -1;
                }
          }
          last_n = n;
          return dir;
        }
};

DirectionEncoder de_right = DirectionEncoder(A2, A1);

void setup() {
    Serial.begin(9600);
}

void loop() {
  
        de_right.update();
    
       // Check if change in direction right wheel
      if (de_right.dir != de_right.last_dir && compensatedRight == false) {
//          Serial.println("Change in direction");
          de_right.last_dir = de_right.dir;
          ticksRight -= compensationTicks;
          compensatedRight = true;
      }
      compensatedRight = false;
      
//      // Count wheel ticks
      n = readSensor(pinA, 4.0);
      if (n != last_n) {
          ticksRight += 1 * de_right.dir;
          Serial.print("right: ");
          Serial.println(ticksRight);
          last_n = n;
      }
      
}
