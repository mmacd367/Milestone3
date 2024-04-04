/*

 MSE 2202 Team 1 Bot Drive System Code
 Language: Arduino
 Author: Mia Macdonald-Walden

 Rev 1  - Initial Version March 20 2024

*/

//  To program and use ESP32-S3
//   
//  Tools->:
//  Board: "Adafruit Feather ESP32-S3 No PSRAM"
//  Upload Speed: "921600"
//  USB CDC On Boot: "Enabled"
//  USB Firmware MSC on Boot: "Disabled"
//  USB DFU On Bot: "Disabled"
//  Upload Mode:"UART0/Hardware CDC"
//  SPU Frequency: "240MHz (WiFi)"
//  Flash Mode: "QIO 80MHz"
//  Flash SIze: "4MB (32Mb)"
//  Partition Scheme: "Default 4MB with spiffs (1.2MB app/1.5MB SPIFFS)"
//  Core Debug Level: "Verbose"
//  PSRAM: 'Disabled"
//  Arduino Runs On: "Core 1"
//  Events Run On: "Core 1"
//
//  To program, press and hold the reset button then press and hold program button, release the reset button then 
//  release the program button 
//

//Debugging Keywords
//

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_TCS34725.h"
#include <MSE2202_Lib.h>


// Encoder structure
struct Encoder {
   const int chanA;                                                            // GPIO pin for encoder channel A
   const int chanB;                                                            // GPIO pin for encoder channel B
   long pos;                                                                   // current encoder position
};

// Port pin constants (motors and encoders)
#define LEFT_MOTOR_A        35                                                 // GPIO35 pin 28 (J35) Motor 1 A
#define LEFT_MOTOR_B        36                                                 // GPIO36 pin 29 (J36) Motor 1 B
#define RIGHT_MOTOR_A       37                                                 // GPIO37 pin 30 (J37) Motor 2 A
#define RIGHT_MOTOR_B       38                                                 // GPIO38 pin 31 (J38) Motor 2 B
#define ENCODER_LEFT_A      15                                                 // left encoder A signal is connected to pin 8 GPIO15 (J15)
#define ENCODER_LEFT_B      16                                                 // left encoder B signal is connected to pin 8 GPIO16 (J16)
#define ENCODER_RIGHT_A     11                                                 // right encoder A signal is connected to pin 19 GPIO11 (J11)
#define ENCODER_RIGHT_B     12                                                 // right encoder B signal is connected to pin 20 GPIO12 (J12)

// Port pin constants (servo motors)
#define SERVO1              38                                                 // 
#define SERVO2              39                                                 //
#define SERVO3              40                                                 // Right pickup arm servo
#define SERVO4              41                                                 // Left pickup arm servo
#define SERVO5              42                                                 // Scoop servo
#define SERVO6                                                                 //
#define PWMCHAN_SERVO1       4 
#define PWMCHAN_SERVO2       5
#define PWMCHAN_SERVO3       6
#define PWMCHAN_SERVO4       7
#define PWMCHAN_SERVO5       

// Port pin constants (other)
#define MODE_BUTTON          0                                                 // GPIO0  pin 27 for Push Button 1
#define MOTOR_ENABLE_SWITCH  3                                                 // DIP Switch S1-1 pulls Digital pin D3 to ground when on, connected to pin 15 GPIO3 (J3)
#define SMART_LED           21                                                 // when DIP Switch S1-4 is on, Smart LED is connected to pin 23 GPIO21 (J21)
#define SMART_LED_COUNT      1 

// Constants
const int cDisplayUpdate = 100;                                                // update interval for Smart LED in milliseconds
const int cPWMRes = 4;                                                         // bit resolution for PWM
const int cMinPWM = 150;                                                       // PWM value for minimum speed that turns motor
const int cMaxPWM = pow(2, cPWMRes) - 1;                                       // PWM value for maximum speed
const int cCountsRev = 1096;                                                   // encoder pulses per motor revolution
const int cReturnTime = 100000;                                                // time for drive system, before returning to base
const int cTurnRadius = 8;                                                     // bot's turning radius
const int cRevDistance = 25.761;                                               // distance traversed by the bot for 1 wheel revolution

// adjustment variables and drive speed
const int cLeftAdjust = 0;                                                     // Amount to slow down left motor relative to right
const int cRightAdjust = 0;                                                    // Amount to slow down right motor relative to left

// Variables (operation)
boolean motorsEnabled = true;                                                  // motors enabled flag
boolean timeUpReturn = false;                                                  // drive timer elapsed flag
unsigned char leftDriveSpeed;                                                  // motor drive speed (0-255)
unsigned char rightDriveSpeed;                                                 // motor drive speed (0-255)
unsigned int robotModeIndex = 0;                                               // state index for run mode
unsigned int driveIndex = 0;                                                   // state index for drive
int Operationflag = 1;
int Pickupflag = 1;                                                            // flag to switch between cases
int SortFlag = 1;
int goodflag = 1;

// Variables (servos)
int startAngleServo1 = 0;  // Initial angle for servo 1
int endAngleServo1 = 90;   // Final angle for servo 2
int startAngleServo2 = 0; // Initial angle for servo 1
int endAngleServo2 = 90;  // Final angle for servo 2
int startAngleServo3 = 0;  // Initial angle for servo 3
int endAngleServo3 = 90;   // Final angle for servo 3
int startAngleServo4 = 0; // Initial angle for servo 4
int endAngleServo4 = 90;  // Final angle for servo 4
int startAngleServo5 = 0; // Initial angle for servo 5
int endAngleServo5 = 180; // Final angle for servo 5
float positionServo1 = startAngleServo1; // Current position of servo 1
float positionServo2 = startAngleServo2; // Current position of servo 2
float positionServo3 = startAngleServo3; // Current position of servo 3
float positionServo4 = startAngleServo4; // Current position of servo 4
float positionServo5 = endAngleServo5; // Current position of servo 5
int speedFactorServo1 = 1; // Speed factor for servo 1
int speedFactorServo2 = 5; // Speed factor for servo 2
int speedFactorServo3 = 1; // Speed factor for servo 3
int speedFactorServo4 = 1; // Speed factor for servo 4
int speedFactorServo5 = 1; // Speed factor for servo 5

// Variables (timers/debounce)
unsigned int  modePBDebounce;                                                  // pushbutton debounce timer count
unsigned long timerCountReturn = 0;                                            // return time counter
unsigned long displayTime;                                                     // heartbeat LED update timer
unsigned long previousMicros;                                                  // last microsecond count
unsigned long currentMicros;                                                   // current microsecond count
unsigned long interval = 20000;                                                // Time interval between servo position updates (in microseconds)


// Declare SK6812 SMART LED object
//   Argument 1 = Number of LEDs (pixels) in use
//   Argument 2 = ESP32 pin number 
//   Argument 3 = Pixel type flags, add together as needed:
//     NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//     NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//     NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//     NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//     NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
Adafruit_NeoPixel SmartLEDs(SMART_LED_COUNT, SMART_LED, NEO_RGB + NEO_KHZ800);

// smart LED brightness for heartbeat
unsigned char LEDBrightnessIndex = 0; 
unsigned char LEDBrightnessLevels[] = {5,15,30,45,60,75,90,105,120,135,150,165,180,195,210,225,240,255,
                                       240,225,210,195,180,165,150,135,120,105,90,75,60,45,30,15};
                             
unsigned int  modeIndicator[6] = {                                             // colours for different modes
   SmartLEDs.Color(255,0,0),                                                   // red - stop
   SmartLEDs.Color(0,255,0),                                                   // green - drive
   SmartLEDs.Color(0,0,255),                                                   // blue - pick up
   SmartLEDs.Color(255,255,0),                                                 // yellow - navigate home
   SmartLEDs.Color(0,255,255),                                                 // cyan - release
   SmartLEDs.Color(255,0,255)                                                  // magenta - empty case
};                                                                            

// Motor, encoder, and IR objects (classes defined in MSE2202_Lib)
Motion Bot = Motion();                                                         // Instance of Motion for motor control
Encoders LeftEncoder = Encoders();                                             // Instance of Encoders for left encoder data
Encoders RightEncoder = Encoders();                                            // Instance of Encoders for right encoder data
 
// function declarations
int degreesToDutyCycle(int deg);
void Pickup();
void Sorting();
void Indicator();

void setup() {
  // config serial comms
  Serial.begin(115200);

   // Set up motors and encoders
   Bot.driveBegin("D1", LEFT_MOTOR_A, LEFT_MOTOR_B, RIGHT_MOTOR_A, RIGHT_MOTOR_B); // set up motors as Drive 1
   LeftEncoder.Begin(ENCODER_LEFT_A, ENCODER_LEFT_B, &Bot.iLeftMotorRunning ); // set up left encoder
   RightEncoder.Begin(ENCODER_RIGHT_A, ENCODER_RIGHT_B, &Bot.iRightMotorRunning ); // set up right encoder

   // Set up SmartLED
   SmartLEDs.begin();                                                          // initialize smart LEDs object (REQUIRED)
   SmartLEDs.clear();                                                          // clear pixel
   SmartLEDs.setPixelColor(0,SmartLEDs.Color(0,0,0));                          // set pixel colors to 'off'
   SmartLEDs.show();                                                           // send the updated pixel colors to the hardware

   pinMode(MOTOR_ENABLE_SWITCH, INPUT_PULLUP);                                 // set up motor enable switch with internal pullup
   pinMode(MODE_BUTTON, INPUT_PULLUP);                                         // Set up mode pushbutton
   modePBDebounce = 0;

   // set up the servo pins as outputs
  pinMode(SERVO1, OUTPUT);
  pinMode(SERVO2, OUTPUT);
  pinMode(SERVO3, OUTPUT);
  pinMode(SERVO4, OUTPUT);
  pinMode(SERVO5, OUTPUT);

  // set up LED channels for each servo
  ledcSetup(PWMCHAN_SERVO1, 40, 14); // pwm channel, frequency and bit resolution
  ledcSetup(PWMCHAN_SERVO2, 40, 14); // pwm channel, frequency and bit resolution
  ledcSetup(PWMCHAN_SERVO3, 40, 14); // pwm channel, frequency and bit resolution
  ledcSetup(PWMCHAN_SERVO4, 40, 14); // pwm channel, frequency and bit resolution
  ledcSetup(PWMCHAN_SERVO5, 40, 14); // pwm channel, frequency and bit resolution


  // atach the LED channels for each servo to different GPIO pins
  ledcAttachPin(SERVO1, PWMCHAN_SERVO1);
  ledcAttachPin(SERVO2, PWMCHAN_SERVO2);
  ledcAttachPin(SERVO3, PWMCHAN_SERVO3);
  ledcAttachPin(SERVO4, PWMCHAN_SERVO4);
  ledcAttachPin(SERVO5, PWMCHAN_SERVO5);
}

void loop() {
  long pos[] = {0, 0};                                                        // current motor positions
   
  currentMicros = micros();                                                   // get current time in microseconds
  if((currentMicros - previousMicros) >= 1000){                               // enter if 1 millisecond has passed since last entry
    previousMicros = currentMicros;                                           // record current time in microseconds 

    // Mode pushbutton debounce and toggle
    if (!digitalRead(MODE_BUTTON)) {                                            // if pushbutton GPIO goes LOW (nominal push)
    
      // Start debounce
      if (modePBDebounce <= 25) {                                               // 25 millisecond debounce time
        modePBDebounce = modePBDebounce + 1;                                    // increment debounce timer count
        if (modePBDebounce > 25) {                                              // if held for at least 25 mS
          modePBDebounce = 1000;                                                // change debounce timer count to 1 second
        }
      }

      if (modePBDebounce >= 1000) {                                             // maintain 1 second timer count until release
        modePBDebounce = 1000;
      }
    }

    else {                                                                      // pushbutton GPIO goes HIGH (nominal release)
      if(modePBDebounce <= 26) {                                                // if release occurs within debounce interval
        modePBDebounce = 0;                                                     // reset debounce timer count
      }
      else {
        modePBDebounce = modePBDebounce + 1;                                    // increment debounce timer count
        if(modePBDebounce >= 1025) {                                            // if pushbutton was released for 25 mS
          modePBDebounce = 0;                                                   // reset debounce timer count
          robotModeIndex++;                                                     // move robot to next mode
          robotModeIndex = robotModeIndex & 7;                                  // keep mode index between 0 and 7
          timeUpReturn = false;                                                 // reset return timer
        }
      }
    }

    // check if drive motors should be powered REMOVE?? NECESSARY???
    motorsEnabled = !digitalRead(MOTOR_ENABLE_SWITCH);                          // if SW1-1 is on (low signal), then motors are enabled\

    // drive and operation modes
    // 0 = Default after power up/reset. Robot is stopped.
    // 1 = Drive 
    // 2 = Pick up gems
    // 3 = Navigate to home base
    // 4 = Open back hatch
    switch(robotModeIndex){
      case 0: // robot stopped
        Bot.Stop("D1");                                                         // Stop the wheels
        LeftEncoder.clearEncoder();                                             // reset left encoder count
        RightEncoder.clearEncoder();                                            // reset right encoder count
        driveIndex = 0;                                                         // set the drive index to 0 (first driving state)
        break;

      case 1: // drive 
        robotModeIndex = 2;
        break;

      case 2: // operate pick up
        if(currentMicros >= interval){
          Pickup();
          previousMillis = currentMillis;
        }
        break;

      case 3: // navigate to home base
        robotModeIndex = 0;
        break;

      case 4: // open back hatch

        break;
    }
    // Update brightness of heartbeat display on SmartLED
    displayTime++;                                                            // count milliseconds
    if (displayTime > cDisplayUpdate) {                                       // when display update period has passed
      displayTime = 0;                                                        // reset display counter
      LEDBrightnessIndex++;                                                   // shift to next brightness level
      if (LEDBrightnessIndex > sizeof(LEDBrightnessLevels)) {                 // if all defined levels have been used
        LEDBrightnessIndex = 0;                                               // reset to starting brightness
      }
      SmartLEDs.setBrightness(LEDBrightnessLevels[LEDBrightnessIndex]);       // set brightness of heartbeat LED
      Indicator();                                                            // update LED
    }
  }
}

// pickup system operation
void Pickup() {
  switch (Pickupflag) {
      case 1:
        positionServo5 -= (endAngleServo5 - startAngleServo5) / (1000.0 / interval) / speedFactorServo5;
        if(positionServo5 <= startAngleServo5){
          Serial.println("test1");
          positionServo5 = startAngleServo5;
          Pickupflag = 2;
        }
        ledcWrite(PWMCHAN_SERVO5, degreesToDutyCycle(positionServo5));
        Serial.printf("%f\n", positionServo5);
        Serial.println("test if servo 5 is written 2 open");
        break;
        
      case 2:
        positionServo3 += (endAngleServo3 - startAngleServo3) / (1000.0 / interval) / speedFactorServo3;
        positionServo4 -= (endAngleServo4 - startAngleServo4) / (1000.0 / interval) / speedFactorServo4;
        if ((positionServo3 >= endAngleServo3) && (positionServo4 <= startAngleServo4)){
          positionServo3 = endAngleServo3;
          positionServo4 = startAngleServo4;
          Pickupflag = 3;
        }
        ledcWrite(PWMCHAN_SERVO3, degreesToDutyCycle(positionServo3));
        ledcWrite(PWMCHAN_SERVO4, degreesToDutyCycle(positionServo4));
        break;
        
      case 3:
        positionServo3 -= (endAngleServo3 - startAngleServo3) / (1000.0 / interval) / speedFactorServo3;
        positionServo4 += (endAngleServo4 - startAngleServo4) / (1000.0 / interval) / speedFactorServo4;
        if ((positionServo3 <= startAngleServo3) && (positionServo4 >= endAngleServo4)) {
          positionServo3 = startAngleServo3;
          positionServo4 = endAngleServo4;
          Pickupflag = 4;
        }
        ledcWrite(PWMCHAN_SERVO3, degreesToDutyCycle(positionServo3));
        ledcWrite(PWMCHAN_SERVO4, degreesToDutyCycle(positionServo4));
        break;
      
      case 4:
        positionServo5 += (endAngleServo5 - startAngleServo5) / (1000.0 / interval) / speedFactorServo5 ;
        if(positionServo5 >= endAngleServo5){
          Serial.println("test2");
          positionServo5 = endAngleServo5;
          Pickupflag = 1;
          Operationflag = 2; // next operation
        }
        ledcWrite(PWMCHAN_SERVO5, degreesToDutyCycle(positionServo5));
        Serial.printf("%f\n", positionServo5);
        Serial.println("test if servo 5 is written 2 close ");
        break;

    }
}
// function to return a dutyCyle that should be written using pwm to the servos
int degreesToDutyCycle(int deg) {
  const long MinimumDC = 400; // Duty Cycle for 0 degrees
  const long MaximumDC = 2100; // Duty Cycle for 180 degrees
  int dutyCycle = map(deg, 0, 180, MinimumDC, MaximumDC); // map from degrees to duty cycle
  return dutyCycle;
}

// Set colour of Smart LED depending on robot mode (and update brightness)
void Indicator() {
  SmartLEDs.setPixelColor(0, modeIndicator[robotModeIndex]);                  // set pixel colors to = mode 
  SmartLEDs.show();                                                           // send the updated pixel colors to the hardware
}
