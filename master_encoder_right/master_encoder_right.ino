/**
This program combines readings from the speed and direction encoders to provide the number and direction of wheel "ticks".
**/

// Global variables
int pinA = A0;
int ticksRight = 0;
int ticksLeft = 0;
int pinB = A3;

int n;
int last_n = LOW;
int n2;
int last_n2 = LOW;

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
DirectionEncoder de_left = DirectionEncoder(A4, A5);


void setup() {
    Serial.begin(9600);
}

int compensationTicks = 20;
int compensatedRight = false;  // This boolean variable indicates whether the speed encoder has been compensated for the delay in the direction encoder.
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
      compensatedRight = false;
      if (de_left.dir != de_left.last_dir && compensatedLeft == false) {
//          Serial.println("Change in direction");
          de_left.last_dir = de_left.dir;
          ticksLeft -= compensationTicks;
          compensatedLeft = true;
      }
      
      // Count wheel ticks right
      n = readSensor(pinA, 4.0);
      if (n != last_n) {
          ticksRight += 1 * de_right.dir;
          Serial.print("right: ");
          Serial.println(ticksRight);
          last_n = n;
      }
      
      n2 = readSensor(pinB, 4.0);
      if (n2 != last_n2) {
          ticksLeft += 1 * de_left.dir;
          Serial.print("left: ");
          Serial.println(ticksLeft);
          last_n2 = n2;
      }
      
      
}
