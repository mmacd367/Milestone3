#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_TCS34725.h"
#include <MSE2202_Lib.h>

// Servo pins
#define SERVO1 38
#define SERVO2 39
#define SERVO6 40

// Pins for things associated with colour sensor
#define cSDA 47                    // GPIO pin for I2C data
#define cSCL 48                    // GPIO pin for I2C clock
#define cTCSLED 14                   // GPIO pin for LED on TCS34725
#define cLEDSwitch 46               // DIP switch S1-2 controls LED on TCS32725    

// Servo channels
#define PWMCHAN_SERVO1 0 
#define PWMCHAN_SERVO2 1
#define PWMCHAN_SERVO6 2

// Constants

// Constants for good gems
const int rgoodmin = 14;
const int ggoodmin = 18;
const int bgoodmin = 11;
const int cgoodmin = 46;

const int rgoodmax = 17;
const int ggoodmax = 24;
const int bgoodmax = 16;
const int cgoodmax = 60;

// Constants for no gems
const int rnonemin = 19, gnonemin = 20, bnonemin = 17, cnonemin = 63;
const int rnonemax = 24, gnonemax = 24, bnonemax = 21, cnonemax = 67;

// TCS34725 colour sensor with 2.4 ms integration time and gain of 4
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);
bool tcsFlag = 0; // Colour sensor active flag

// Colour sensor variables
uint16_t r, g, b, c;                                // RGBC values from TCS34725

// Start and end angles for each servo
int startAngleServo1 = 0;  // Initial angle for servo 1
int endAngleServo1 = 90;   // Final angle for servo 1
int startAngleServo2 = 0; // Initial angle for servo 2
int endAngleServo2 = 90;  // Final angle for servo 2
int startAngleServo6 = 0; // Initial angle for servo 6
int endAngleServo6 = 80; // Final angle for servo 6

// Timer variables
unsigned long FullTimer = 120000; // Total sorter time in milliseconds
unsigned long currentMillis;
unsigned long previousMillis = 0; // Previous millisecond count
unsigned long interval = 20; // Update time in milliseconds

// Starting positions for all servos
float positionServo1 = endAngleServo1; // Current position of servo 1
float positionServo2 = endAngleServo2; // Current position of servo 2
float positionServo6 = endAngleServo6; // Current position of servo 6

// Speed factors for all servos
int speedFactorServo1 = 1; // Speed factor for servo 1
int speedFactorServo2 = 5; // Speed factor for servo 2

// Sort flag
int Sortflag = 1;
bool goodflag = false;
bool sampleAgain = true;

// Function declarations
int degreesToDutyCycle(int deg);
void Sort();

void setup() {
  // Configuring serial communication
  Serial.begin(115200);

  // Setting up LED channels for each servo
  ledcSetup(PWMCHAN_SERVO1, 40, 14); // PWM channel, frequency, and bit resolution
  ledcSetup(PWMCHAN_SERVO2, 40, 14); // PWM channel, frequency, and bit resolution
  ledcSetup(PWMCHAN_SERVO6, 40, 14); // PWM channel, frequency, and bit resolution

  // Attaching the LED channels for each servo to different GPIO pins
  ledcAttachPin(SERVO1, PWMCHAN_SERVO1);
  ledcAttachPin(SERVO2, PWMCHAN_SERVO2);
  ledcAttachPin(SERVO6, PWMCHAN_SERVO6);
  
  // Setting the original positions of the sorting servos
  while(positionServo1 > startAngleServo1){
    positionServo1 -= 1;
    ledcWrite(PWMCHAN_SERVO1, degreesToDutyCycle(positionServo1));
  }

  while(positionServo2 > startAngleServo2){
    positionServo2 -= 1;
    ledcWrite(PWMCHAN_SERVO2, degreesToDutyCycle(positionServo2));
  }

  while(positionServo6 > startAngleServo6){
    positionServo6 -=1;
    ledcWrite(PWMCHAN_SERVO6, degreesToDutyCycle(positionServo6));
  }

  // Set up colour sensor
  Wire.setPins(cSDA, cSCL);                           // Set I2C pins for TCS34725
  pinMode(cTCSLED, OUTPUT);                           // Configure GPIO to control LED on TCS34725
  pinMode(cLEDSwitch, INPUT_PULLUP);                  // Configure GPIO to set state of TCS34725 LED 

  // Connect to TCS34725 colour sensor
  if (tcs.begin()) {
    Serial.println("Found TCS34725 colour sensor");
    tcsFlag = true;
  } 
  else {
    Serial.println("No TCS34725 found ... check your connections");
    tcsFlag = false;
  }
}

void loop() {
  currentMillis = millis(); // Get current time
  unsigned long elapsedTime = currentMillis - previousMillis; // Get elapsed time since last loop

  // Sample the color of the gems
  if(currentMillis < FullTimer){
    if(sampleAgain){
      // Getting colour sensor data
      if (tcsFlag) {                                      // If colour sensor initialized
        tcs.getRawData(&r, &g, &b, &c);                   // Get raw RGBC values
      }
      Serial.printf("R: %d, G: %d, B: %d, C %d\n", r, g, b, c);
      digitalWrite(cTCSLED, !digitalRead(cLEDSwitch));    // Turn on onboard LED if switch state is low (on position)
      sampleAgain = false;    
    }
    
    // Check gem color and sort
    if(( r >= rgoodmin && r <= rgoodmax) && (g >= ggoodmin && g <= ggoodmax) && (b >= bgoodmin && b <= bgoodmax) && (c >= cgoodmin && c <= cgoodmax)){
      if(elapsedTime >= interval){
        goodflag = true;
        Sort();
        previousMillis = currentMillis;
      }
    }
    else if (( r >= rnonemin && r <= rnonemax) && (g >= gnonemin && g <= gnonemax) && (b >= bnonemin && b <= bnonemax) && (c >= cnonemin && c <= cnonemax)){
      sampleAgain = true;
    }
    else{
      if(elapsedTime >= interval){
        goodflag = false; // Bad gem
        Sort();
        previousMillis = currentMillis;
      }  
    }  
  }
  
  // If the full timer is reached, close the last servo
  while(currentMillis >= FullTimer){
    positionServo6 += 1;
    ledcWrite(PWMCHAN_SERVO6, degreesToDutyCycle(positionServo6));
    Serial.println("Test print end servo");
  }
}

// Function to handle sorting of gems
void Sort(){
  if (goodflag){
      switch (Sortflag) {
      case 1:
        // Open servo 1
        positionServo1 += ((endAngleServo1 - startAngleServo1) / (1000.0 / interval)) * speedFactorServo1;
        if(positionServo1 >= endAngleServo1){
          positionServo1 = endAngleServo1;
          Sortflag = 2;
        }
        ledcWrite(PWMCHAN_SERVO1, degreesToDutyCycle(positionServo1));
        Serial.printf("%f\n", positionServo1);
        Serial.println("Test if servo 1 is open");
        break;
        
      case 2:
        // Open servo 2
        positionServo2 += ((endAngleServo2 - startAngleServo2) / (1000.0 / interval)) * speedFactorServo2;
        if (positionServo2 >= endAngleServo2){
          positionServo2 = endAngleServo2;
          Sortflag = 3;
        }
        ledcWrite(PWMCHAN_SERVO2, degreesToDutyCycle(positionServo2));
        break;
        
      case 3:
        // Close servo 2
        positionServo2 -= ((endAngleServo2 - startAngleServo2) / (1000.0 / interval)) * speedFactorServo2;
        if (positionServo2 <= startAngleServo2) {
          positionServo2 = startAngleServo2;
          Sortflag = 4;
        }
        ledcWrite(PWMCHAN_SERVO2, degreesToDutyCycle(positionServo2));
        break;
      
      case 4:
        // Close servo 1
        positionServo1 -= ((endAngleServo1 - startAngleServo1) / (1000.0 / interval)) * speedFactorServo1 ;
        if(positionServo1 <= startAngleServo1){
          positionServo1 = startAngleServo1;
          sampleAgain = true;
          Sortflag = 1;
        }
        ledcWrite(PWMCHAN_SERVO1, degreesToDutyCycle(positionServo1));
        Serial.printf("%f\n", positionServo1);
        Serial.println("Test if servo 1 is close");
        break;
      }
    }
    else{
      switch(Sortflag) {
      case 1:
        // Open servo 1
        positionServo1 += ((endAngleServo1 - startAngleServo1) / (1000.0 / interval)) * speedFactorServo1;
        if(positionServo1 >= endAngleServo1){
          positionServo1 = endAngleServo1;
          Sortflag = 2;
        }
        ledcWrite(PWMCHAN_SERVO1, degreesToDutyCycle(positionServo1));
        Serial.printf("%f\n", positionServo1);
        Serial.println("Test if servo 1 is open");
        break;
        
      case 2:
        // Close servo 1
        positionServo1 -= ((endAngleServo1 - startAngleServo1) / (1000.0 / interval)) * speedFactorServo1;
        if (positionServo1 <= startAngleServo1){
          positionServo1 = startAngleServo1;
          Sortflag = 3;
        }
        ledcWrite(PWMCHAN_SERVO1, degreesToDutyCycle(positionServo1));
        break;
        
      case 3:
        // Open servo 2
        positionServo2 += ((endAngleServo2 - startAngleServo2) / (1000.0 / interval)) * speedFactorServo2;
        if (positionServo2 >= endAngleServo2) {
          positionServo2 = endAngleServo2;
          Sortflag = 4;
        }
        ledcWrite(PWMCHAN_SERVO2, degreesToDutyCycle(positionServo2));
        break;
      
      case 4:
        // Close servo 2
        positionServo2 -= ((endAngleServo2 - startAngleServo2) / (1000.0 / interval)) * speedFactorServo2 ;
        if(positionServo2 <= startAngleServo2){
          positionServo2 = startAngleServo2;
          sampleAgain = true;
          Sortflag = 1;
        }
        ledcWrite(PWMCHAN_SERVO2, degreesToDutyCycle(positionServo2));
        Serial.printf("%f\n", positionServo2);
        Serial.println("Test if servo 2 is close");
        break;
      }  
  }
}

// Function to return a duty cycle that should be written using PWM to the servos
int degreesToDutyCycle(int deg) {
  const long MinimumDC = 400; // Duty Cycle for 0 degrees
  const long MaximumDC = 2100; // Duty Cycle for 180 degrees
  int dutyCycle = map(deg, 0, 180, MinimumDC, MaximumDC); // Map from degrees to duty cycle
  return dutyCycle;
}