#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_TCS34725.h"
#include <MSE2202_Lib.h>

// Servo pins
#define SERVO1 38
#define SERVO2 39
#define SERVO6 2

// Pins for things associated with colour sensor

#define cSDA 47                    // GPIO pin for I2C data
#define cSCL 48                    // GPIO pin for I2C clock
#define cTCSLED 14                   // GPIO pin for LED on TCS34725
#define cLEDSwitch 46               // DIP switch S1-2 controls LED on TCS32725    

// servo channels
#define PWMCHAN_SERVO1 0 
#define PWMCHAN_SERVO2 1
#define PWMCHAN_SERVO6 2

// Constants

// for good gems
const int rgoodmin = 14;
const int ggoodmin = 18;
const int bgoodmin = 11;
const int cgoodmin = 46;


const int rgoodmax = 17;
const int ggoodmax = 24;
const int bgoodmax = 16;
const int cgoodmax = 60;

// for no gems
const int rnonemin = 19, gnonemin = 20, bnonemin = 17, cnonemin = 63;
const int rnonemax = 24, gnonemax = 24, bnonemax = 21, cnonemax = 67;


// start and end angles for each servo
int startAngleServo1 = 0;  // Initial angle for servo 1
int endAngleServo1 = 90;   // Final angle for servo 2
int startAngleServo2 = 0; // Initial angle for servo 1
int endAngleServo2 = 90;  // Final angle for servo 2
int startAngleServo6 = 0; // Initial angle for servo 6
int endAngleServo6 = 80; // Final angle for servo 6

// timer variables
// Full Timer
unsigned long FullTimer = 120000 //miliseconds total sorter time
unsigned long currentMillis;
unsigned long previousMillis = 0; //previous milllisecond count
unsigned long interval = 20; // update time in miliseconds

// starting positions for all servos
float positionServo1 = endAngleServo1; // Current position of servo 1
float positionServo2 = endAngleServo2; // Current position of servo 2
float positionServo6 = endAngleServo6; // Current position of servo 6

// speed factors for all servos
int speedFactorServo1 = 1; // Speed factor for servo 1
int speedFactorServo2 = 5; // Speed factor for servo 2

// sort flag
int Sortflag = 1;
bool goodflag = false;
bool sampleAgain = true;

// function declarations
int degreesToDutyCycle(int deg);
void Sort();


void setup() {
  // put your setup code here, to run once:
   // put your setup code here, to run once:
  // config serial comms
  Serial.begin(115200);

  // set up the servo pins as outputs
  pinMode(SERVO1, OUTPUT);
  pinMode(SERVO2, OUTPUT);
  pinMode(SERVO6, OUTPUT);

  // set up LED channels for each servo
  ledcSetup(PWMCHAN_SERVO1, 40, 14); // pwm channel, frequency and bit resolution
  ledcSetup(PWMCHAN_SERVO2, 40, 14); // pwm channel, frequency and bit resolution
  ledcSetup(PWMCHAN_SERVO6, 40, 14); // pwm channel, frequency and bit resolution

  // atach the LED channels for each servo to different GPIO pins
  ledcAttachPin(SERVO1, PWMCHAN_SERVO1);
  ledcAttachPin(SERVO2, PWMCHAN_SERVO2);
  ledcAttachPin(SERVO6, PWMCHAN_SERVO6);
  
  // set the original positions of the sorting servos
  while(positionServo1 >= startAngleServo1){
    positionServo1 -= 1;
    ledcWrite(PWMCHAN_SERVO1, degreesToDutyCycle(positionServo1));
  }

  while(positionServo2 >= startAngleServo2){
    positionServo2 -= 1;
    ledcWrite(PWMCHAN_SERVO2, degreesToDutyCycle(positionServo2));
  }

  while(positionServo6 >= startAngleServo){
    positionServo6 -=1;
    ledcWrite(PWMCHAN_SERVO6, degreesToDutyCycle(positionServo6));
  }

   // Set up colour sensor
  Wire.setPins(cSDA, cSCL);                           // set I2C pins for TCS34725
  pinMode(cTCSLED, OUTPUT);                           // configure GPIO to control LED on TCS34725
  pinMode(cLEDSwitch, INPUT_PULLUP);                  // configure GPIO to set state of TCS34725 LED 

  // Connect to TCS34725 colour sensor
  if (tcs.begin()) {
    Serial.printf("Found TCS34725 colour sensor\n");
    tcsFlag = true;
  } 
  else {
    Serial.printf("No TCS34725 found ... check your connections\n");
    tcsFlag = false;
  }

}

void loop() {
  currentMillis = millis(); // get current time
  unsigned long elapsedTime = currentMillis - previousMillis; // get elapsed time since last loop
  // sample the color of the gems
  if(currentMillis <= FullTimer){
    if(sampleAgain){
      // getting colour semsor data
      if (tcsFlag) {                                      // if colour sensor initialized
      tcs.getRawData(&r, &g, &b, &c);                   // get raw RGBC values
      }
      Serial.printf("R: %d, G: %d, B: %d, C %d\n", r, g, b, c);
      digitalWrite(cTCSLED, !digitalRead(cLEDSwitch));    // turn on onboard LED if switch state is low (on position)
      sampleAgain = false;    
    }
    
      // if else statement to branch to Sort function
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
      goodflag = false;
      Sort();
      previousMillis = currentMillis;
    }  
  }
  
  while(positionServo6 <= endAngleServo6){
    positionServo6 +=1;
    ledcWrite(PWMCHAN_SERVO6, degreesToDutyCycle(positionServo6));
  }
  
}

void Sort(){
  if (goodflag){
      switch (Sortflag) {
      case 1:
        positionServo1 += ((endAngleServo1 - startAngleServo1) / (1000.0 / interval)) * speedFactorServo1;
        if(positionServo1 >= endAngleServo1){
          Serial.println("test1");
          positionServo1 = endAngleServo1;
          Sortflag = 2;
        }
        ledcWrite(PWMCHAN_SERVO1, degreesToDutyCycle(positionServo1));
        Serial.printf("%f\n", positionServo1);
        Serial.println("test if servo 1 is written 2 open");
        break;
        
      case 2:
        positionServo2 += ((endAngleServo2 - startAngleServo2) / (1000.0 / interval)) * speedFactorServo2;
        if (positionServo2 >= endAngleServo2){
          positionServo2 = endAngleServo2;
          Sortflag = 3;
        }
        ledcWrite(PWMCHAN_SERVO2, degreesToDutyCycle(positionServo2));
        break;
        
      case 3:
        positionServo2 -= ((endAngleServo2 - startAngleServo2) / (1000.0 / interval)) * speedFactorServo2;
        if (positionServo2 <= startAngleServo2) {
          positionServo2 = startAngleServo2;
          Sortflag = 4;
        }
        ledcWrite(PWMCHAN_SERVO2, degreesToDutyCycle(positionServo2));
        break;
      
      case 4:
        positionServo1 -= ((endAngleServo1 - startAngleServo1) / (1000.0 / interval)) * speedFactorServo1 ;
        if(positionServo1 <= startAngleServo1){
          Serial.println("test2");
          positionServo1 = startAngleServo1;
          sampleAgain = true;
          Sortflag = 1;
        }
        ledcWrite(PWMCHAN_SERVO1, degreesToDutyCycle(positionServo1));
        Serial.printf("%f\n", positionServo1);
        Serial.println("test if servo 1 is written 2 close ");
        break;
      }
    }
    else{
      switch(Sortflag) {
      case 1:
        positionServo1 += ((endAngleServo1 - startAngleServo1) / (1000.0 / interval)) * speedFactorServo1;
        if(positionServo1 >= endAngleServo1){
          Serial.println("test1");
          positionServo1 = endAngleServo1;
          Sortflag = 2;
        }
        ledcWrite(PWMCHAN_SERVO1, degreesToDutyCycle(positionServo1));
        Serial.printf("%f\n", positionServo1);
        Serial.println("test if servo 1 is written 2 open");
        break;
        
      case 2:
        positionServo1 -= ((endAngleServo1 - startAngleServo1) / (1000.0 / interval)) * speedFactorServo1;
        if (positionServo1 <= startAngleServo1){
          positionServo1 = startAngleServo1;
          Sortflag = 3;
        }
        ledcWrite(PWMCHAN_SERVO1, degreesToDutyCycle(positionServo1));
        break;
        
      case 3:
        positionServo2 += ((endAngleServo2 - startAngleServo2) / (1000.0 / interval)) * speedFactorServo2;
        if (positionServo2 >= endAngleServo2) {
          positionServo2 = endAngleServo2;
          Sortflag = 4;
        }
        ledcWrite(PWMCHAN_SERVO2, degreesToDutyCycle(positionServo2));
        break;
      
      case 4:
        positionServo1 -= ((endAngleServo1 - startAngleServo1) / (1000.0 / interval)) * speedFactorServo1 ;
        if(positionServo1 <= startAngleServo1){
          Serial.println("test2");
          positionServo1 = startAngleServo1;
          Sortflag = 1;
          sampleAgain = true;
        }
        ledcWrite(PWMCHAN_SERVO1, degreesToDutyCycle(positionServo1));
        Serial.printf("%f\n", positionServo1);
        Serial.println("test if servo 1 is written 2 close ");
        break;
      }  
  }
}