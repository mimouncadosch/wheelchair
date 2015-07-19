#include <Thread.h>
#include <ThreadController.h>
#include <TimerOne.h>

// Above threshold: HIGH, Below threshold: LOW
int threshold = 2.5;

// Read optical IR sensor, convert analog signal to digital value (0 or 1)
int readSensor(int pin) {
  float voltage = analogRead(pin) * (5.0 / 1023.0);  // Voltage
  return (voltage >= threshold) ? HIGH : LOW;
}

class DirectionEncoderThread: public Thread {
   public: 
      // Direction of wheel rotation: +1 forward; -1 backward
      int dir;  
      // Pins from which analog values will be read
      int pinA;
      int pinB;
      // Sensor reading values
      int n;              // New sensor reading
      int last_n;         // Previous sensor reading
      String threadName;  // Thread name
      
      void run() {
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
          Serial.println(threadName + " :" + dir);
          runned();
      }       
};

class SpeedEncoderThread: public Thread {
    public: 
    int revs;  // Revolutions
    int pin;
    int n;
    int last_n;
    String threadName;
    
    void run() {
        n = readSensor(pin);
        if (n != last_n) {
            revs += 1;
            Serial.println(threadName + " :" + revs);
            last_n = n;
          }
          runned();
    }
};

DirectionEncoderThread directionEncoder = DirectionEncoderThread();
SpeedEncoderThread speedEncoder = SpeedEncoderThread();

ThreadController controller = ThreadController();

void timerCallback() {
    controller.run(); 
}

void setup() {
    Serial.begin(9600);
    
    // Set up speed encoder class variables
    directionEncoder.dir = 0;
    directionEncoder.pinA = A0;
    directionEncoder.pinB = A1;
    directionEncoder.last_n = LOW;
    directionEncoder.threadName = "right wheel direction: ";
    directionEncoder.setInterval(100);
    
    // Set up direction encoder class variables
    speedEncoder.revs = 0;
    speedEncoder.pin = A3;
    speedEncoder.last_n = LOW;
    speedEncoder.threadName = "right_wheel speed: ";
    directionEncoder.setInterval(10);
    
    // Add the Threads to our ThreadController
    controller.add(&directionEncoder);
//    controller.add(&speedEncoder);
    
    Timer1.initialize(20000);
    Timer1.attachInterrupt(timerCallback);
    Timer1.start();
}

void loop(){
  // Get the fresh readings
//  Serial.print("Speed Encoder Thread: ");
//  Serial.println(speedEncoder.dir);
}

