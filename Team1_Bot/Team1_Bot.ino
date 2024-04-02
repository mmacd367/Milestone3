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

// Include files into program
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_TCS34725.h>
#include <MSE2202_Lib.h>
#include <ESP32Servo.h>

// Encoder structure
struct Encoder {
   const int chanA;                                                            // GPIO pin for encoder channel A
   const int chanB;                                                            // GPIO pin for encoder channel B
   long pos;                                                                   // current encoder position
};

// Port pin constants
#define LEFT_MOTOR_A        35                                                 // GPIO35 pin 28 (J35) Motor 1 A
#define LEFT_MOTOR_B        36                                                 // GPIO36 pin 29 (J36) Motor 1 B
#define RIGHT_MOTOR_A       37                                                 // GPIO37 pin 30 (J37) Motor 2 A
#define RIGHT_MOTOR_B       38                                                 // GPIO38 pin 31 (J38) Motor 2 B
#define ENCODER_LEFT_A      15                                                 // left encoder A signal is connected to pin 8 GPIO15 (J15)
#define ENCODER_LEFT_B      16                                                 // left encoder B signal is connected to pin 8 GPIO16 (J16)
#define ENCODER_RIGHT_A     11                                                 // right encoder A signal is connected to pin 19 GPIO11 (J11)
#define ENCODER_RIGHT_B     12                                                 // right encoder B signal is connected to pin 20 GPIO12 (J12)

#define SERVO_1              1                                                 // Servo for flow gate in sorting system
#define SERVO_2              2                                                 // Servo for sorting door in sorting system
#define SERVO_3             40                                                 // Servo for left pickup arm 
#define SERVO_4             41                                                 // Servo for right pickup arm
#define SERVO_5             42                                                 // Servo for scoop
#define SERVO_6             43                                                 // Servo for release hatch 

#define MODE_BUTTON         0                                                  // GPIO0  pin 27 for Push Button 1
#define MOTOR_ENABLE_SWITCH 3                                                  // DIP Switch S1-1 pulls Digital pin D3 to ground when on, connected to pin 15 GPIO3 (J3)
#define SMART_LED           21                                                 // when DIP Switch S1-4 is on, Smart LED is connected to pin 23 GPIO21 (J21)
#define SMART_LED_COUNT     1                                                  // number of smart led's 
#define TRIG_PIN            6                                                  // GIPO pin for the ultrasonic sensor (trig)
#define ECHO_PIN            7                                                  // GIPO pin for the ultrasonic sensor (echo)
#define PI                  3.1415926535897932384626433832795                  // const. value for PI

// Constants
const int cDisplayUpdate = 100;                                                // update interval for Smart LED in milliseconds
const int cPWMRes = 4;                                                         // bit resolution for PWM
const int cMinPWM = 150;                                                       // PWM value for minimum speed that turns motor
const int cMaxPWM = pow(2, cPWMRes) - 1;                                       // PWM value for maximum speed
const int cCountsRev = 1096;                                                   // encoder pulses per motor revolution
const int cReturnTime = 100000;                                                // time for drive system, before returning to base
const long cTurnRadius = 5.2;                                                  // bot's turning radius
const int cRevDistance = 26.075;                                               // distance traversed by the bot for 1 wheel revolution
const int cSDA = 47;                                                           // GPIO pin for I2C data
const int cSCL = 48;                                                           // GPIO pin for I2C clock
const int cTCSLED = 14;                                                        // GPIO pin for LED on TCS34725
const int cLEDSwitch = 46;                                                     // DIP switch S1-2 controls LED on TCS32725  

// adjustment variables and drive speed
const int cLeftAdjust = 1;                                                     // Amount to slow down left motor relative to right
const int cRightAdjust = 0;                                                    // Amount to slow down right motor relative to left

// Variables
boolean motorsEnabled = true;                                                  // motors enabled flag
boolean timeUpReturn = false;                                                  // drive timer elapsed flag
boolean turnDir = false;                                                       // index to indicate what the turn direction should be (left is false, right is true);
boolean wait = true;                                                           // flag to indicate whether the ultrasonic sensor must wait
boolean sample = false;                                                        // flag to indicate whether the colour sensor has sampled 
boolean tcsFlag;

unsigned char leftDriveSpeed;                                                  // motor drive speed (0-255)
unsigned char rightDriveSpeed;                                                 // motor drive speed (0-255)
unsigned int robotModeIndex = 0;                                               // state index for run mode
unsigned int driveIndex = 0;                                                   // state index for drive
unsigned int pickupIndex = 0;                                                  // state index for pickup
unsigned int homeIndex = 0;                                                    // state index for return drive
unsigned int  modePBDebounce;                                                  // pushbutton debounce timer count
unsigned long timerCountReturn = 0;                                            // return time counter
unsigned int turnNo = 0;                                                       // value to indicate how many turns in a direction have been made

unsigned long displayTime;                                                     // heartbeat LED update timer
unsigned long previousMicros;                                                  // last microsecond count
unsigned long currentMicros;                                                   // current microsecond count
unsigned long interval = 20000;                                                                                                                                                                                                    // Time interval between servo position updates (in milliseconds)

long xFromBase;                                                                // current x distance from the base
long yFromBase;                                                                // current y distance from the base
long totalYFromBase;                                                           // total y distance from the base
float us_Duration;                                                             // duration of the ultrasonic signal
float cm_Distance;                                                             // conversion from the ultrasonic signal to distance in cm
int sortMode;                                                                  // the current mode (good, bad, or nothing)

// Variables for servo motors
int startAngleServo1 = 70;                                                     // Initial angle for servo 1
int endAngleServo1 = 180;                                                      // Final angle for servo 1
int startAngleServo2 = 79;                                                     // Initial angle for servo 2
int endAngleServo2 = 180;                                                      // Final angle for servo 2
int startAngleServo3 = 180;                                                     // Initial angle for servo 2
int endAngleServo3 = 0; 
int startAngleServo4 = 0;                                                     // Initial angle for servo 2
int endAngleServo4 = 180; 
int startAngleServo5 = 0;
int endAngleServo5 = 180;
int startAngleServo6 = 0;
int endAngleServo6 = 180;

float positionServo1 = startAngleServo1;                                       // Current position of servo 1
float positionServo2 = startAngleServo2;                                       // Current position of servo 2
float positionServo3 = startAngleServo3;                                       // Current position of servo 3
float positionServo4 = startAngleServo4;                                       // Current position of servo 4
float positionServo5 = startAngleServo5;                                       // Current position of servo 5
float positionServo6 = startAngleServo6;                                       // Current position of servo 6

float speedFactorServo1 = 1.0;                                                 // Speed factor for servo 1
float speedFactorServo2 = 1.0;                                                 // Speed factor for servo 2
float speedFactorServo3 = 0.1;                                                 // Speed factor for servo 3
float speedFactorServo4 = 0.1;                                                 // Speed factor for servo 4
float speedFactorServo5 = 0.1;                                                 // Speed factor for servo 5
float speedFactorServo6 = 0.1;                                                 // Speed factor for servo 6

// state for the sorting system's switch statement
enum colourState {
  SERVO1_FORWARD,
  SERVO1_REVERSE,
  SERVO2_FORWARD,
  SERVO2_REVERSE,
  SAMPLE_AGAIN,
};
colourState ccurrentState = SERVO1_FORWARD;

enum pickupState {
  SERVO5_CLOSE,
  SERVO34_FORWARD,
  SERVO34_REVERSE,
  SERVO5_OPEN,
};

pickupState pcurrentState = SERVO5_CLOSE;

// the current rgbc parameters
uint16_t r, g, b, c; 

// limits for rgb
int rgoodmin = 14;
int ggoodmin = 18;
int bgoodmin = 11;
int cgoodmin = 46;

int rgoodmax = 17;
int ggoodmax = 24;
int bgoodmax = 16;
int cgoodmax = 60;

int rnothingmin = 18;
int rnothingmax = 23;
int gnothingmin = 20;
int gnothingmax = 24;
int bnothingmin = 17;
int bnothingmax = 21;
int cnothingmin = 62;
int cnothingmax = 68;

//TCS434725 Set up
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);

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

// Motor and encoder objects (classes defined in MSE2202_Lib)
Motion Bot = Motion();                                                         // Instance of Motion for motor control
Encoders LeftEncoder = Encoders();                                             // Instance of Encoders for left encoder data
Encoders RightEncoder = Encoders();                                            // Instance of Encoders for right encoder data
 
// servo objects
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo6;

void setup() {
   // Set up motors and encoders
   Bot.driveBegin("D1", LEFT_MOTOR_A, LEFT_MOTOR_B, RIGHT_MOTOR_A, RIGHT_MOTOR_B); // set up motors as Drive 1
   LeftEncoder.Begin(ENCODER_LEFT_A, ENCODER_LEFT_B, &Bot.iLeftMotorRunning );      // set up left encoder
   RightEncoder.Begin(ENCODER_RIGHT_A, ENCODER_RIGHT_B, &Bot.iRightMotorRunning );  // set up right encoder

   // Set up servos 
   Bot.servoBegin("S1", SERVO_1);
   Bot.servoBegin("S2", SERVO_2);
   servo3.attach(SERVO_3);
   servo4.attach(SERVO_4);
   servo5.attach(SERVO_5);
   servo6.attach(SERVO_6);

   // Set up SmartLED
   SmartLEDs.begin();                                                          // initialize smart LEDs object (REQUIRED)
   SmartLEDs.clear();                                                          // clear pixel
   SmartLEDs.setPixelColor(0,SmartLEDs.Color(0,0,0));                          // set pixel colors to 'off'
   SmartLEDs.show();                                                           // send the updated pixel colors to the hardware
   
   Wire.setPins(cSDA, cSCL);                                                   // set I2C pins for TCS34725
   pinMode(cTCSLED, OUTPUT);                                                   // configure GPIO to control LED on TCS34725
   pinMode(cLEDSwitch, INPUT_PULLUP);                                          // configure GPIO to set state of TCS34725 LED 

  // Connect to TCS34725 colour sensor
  if (tcs.begin()) {
    Serial.printf("Found TCS34725 colour sensor\n");
    tcsFlag = true;
  } 
  else {
    Serial.printf("No TCS34725 found ... check your connections\n");
    tcsFlag = false;
  }

  digitalWrite(cTCSLED, !digitalRead(cLEDSwitch));    // turn on onboard LED if switch state is low (on position)
   pinMode(MOTOR_ENABLE_SWITCH, INPUT_PULLUP);                                 // set up motor enable switch with internal pullup
   pinMode(MODE_BUTTON, INPUT_PULLUP);                                         // Set up mode pushbutton
   modePBDebounce = 0;                                                         // reset debounce timer count
}

void loop() {
  long pos[] = {0, 0};                                                        // current motor positions
   
  currentMicros = micros();                                                   // get current time in microseconds
  if((currentMicros - previousMicros) >= 1000){                               // enter if 1 millisecond has passed since last entry
    previousMicros = currentMicros;                                           // record current time in microseconds 

    // track 100 second (100000 milliseconds)
    timerCountReturn++;                                                       // increment the return timer counter
    if(timerCountReturn > cReturnTime){                                       // if the returm timer counter has counter the correct time
      timerCountReturn = 0;                                                   // reset the value of the timer counter
      timeUpReturn = true;                                                    // set the flag for the timer to true (time is up)
    }


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
        Serial.println("drive state");
        robotModeIndex = 2;
        break;

      case 2: // operate pick up
        pickup();
         robotModeIndex = 3;
        break;

      case 3: // navigate to home base
        Serial.println("navigate to home");
        robotModeIndex = 4;
        break;

      case 4: // open back hatch
        positionServo6 += (endAngleServo6 - startAngleServo6) / (1000.0 / interval) / speedFactorServo6; // update the speed position of the servo gradually
        if (positionServo6 >= endAngleServo6){                                                           // if the servo has reached its end position
          positionServo6 = endAngleServo6;                                                               // set the servo to its end position (in case of overshooting position)
          robotModeIndex = 0;                                                                            // stop bot operations
        }
        servo4.write(positionServo6);                                                                    // update the servo's position 
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
    Sorting();
  }
}

void pickup(){
  // only perform servo movement every 20ms due to max speed of servos
  if (currentMicros >= interval) {
    switch (pcurrentState) {                                                  // flag to indicate the current movement mode of the servos
      
      // close the scoop servos onto the gems in front
      case SERVO5_CLOSE:
        positionServo5 += (endAngleServo5 - startAngleServo5)/ (1000.0 / interval) / speedFactorServo5;
        if(positionServo5 >= endAngleServo5){
          Serial.println("test1");
          positionServo5 = endAngleServo5;
          pcurrentState = SERVO34_FORWARD;
        }
        servo5.write(positionServo5);
        break;
        
      // move the arm servos up to dump the gems in the scoop into the chassis
      case SERVO34_FORWARD:
        positionServo3 += (endAngleServo3 - startAngleServo3) / (1000.0 / interval) / speedFactorServo3;
        positionServo4 += (endAngleServo4 - startAngleServo4) / (1000.0 / interval) / speedFactorServo4;
        if ((positionServo3 <= endAngleServo3) && (positionServo4 >= endAngleServo4)){
          positionServo3 = endAngleServo3;
          positionServo4 = endAngleServo4;
          pcurrentState = SERVO34_REVERSE;
        }
        servo3.write(positionServo3);
        servo4.write(positionServo4);
        break;
      
      // reset the arm servos to put arm back on the ground
      case SERVO34_REVERSE:
        positionServo3 -= (endAngleServo3 - startAngleServo3) / (1000.0 / interval) / speedFactorServo3;
        positionServo4 -= (endAngleServo4 - startAngleServo4) / (1000.0 / interval) / speedFactorServo4;
        if ((positionServo3 >= startAngleServo3) && (positionServo4 <= startAngleServo4)) {
          positionServo3 = startAngleServo3;
          positionServo4 = startAngleServo4;
          pcurrentState = SERVO5_OPEN;
        }
        servo3.write(positionServo3);
        servo4.write(positionServo4);
        break;
      
      // re-open the arm servos to collect more gems
      case SERVO5_OPEN:
        positionServo5 -= (endAngleServo5 - startAngleServo5) / (1000.0 / interval) / speedFactorServo5;
        if(positionServo5 <= startAngleServo5){
          Serial.println("test2");
          positionServo5 = startAngleServo5;
          pcurrentState = SERVO5_CLOSE;
        }
        servo5.write(positionServo5);
        break;
    }
  }  
}

// sorting system operation
void Sorting() {
  if(!sample){                                        // if a gem isn't currently being sorted (a gem hasn't been sampled yet)

    tcs.getRawData(&r, &g, &b, &c);                   // get raw RGBC values

    if((rgoodmin <= r && r <= rgoodmax) && (ggoodmin <= g && g <= ggoodmax) && (bgoodmin <= b && b <= bgoodmax) && (cgoodmin <= c && c <= cgoodmax)){
      // Gem is good
      sortMode = 0; // Assuming sortMode = 0 means 'good gem'
      sample = true;
    }

    else if(!(rgoodmin <= r || r <= rgoodmax) || !(ggoodmin <= g || g <= ggoodmax) || !(bgoodmin <= b || b <= bgoodmax) || !(cnothingmin <= c || c <= cnothingmax)){
      // Gem is bad
      sortMode = 1; // Assuming sortMode = 1 means 'bad gem'
      sample = true;
    }

    else if((rnothingmin <= r && r <= rnothingmax) && (gnothingmin <= g && g <= gnothingmax) && (bnothingmin <= b && b <= bnothingmax) && (cnothingmin <= c && c <= cnothingmax)){
      // Gem does not exist or condition for 'no gem'
      sortMode = 2; // Assuming sortMode = 2 means 'no gem or condition not met'
      sample = false;
    }

  }

  if(currentMicros >= interval){                      // if sufficient time has elapsed (enough for the servos to move)

    //if the gem is good (green)
    if (sortMode == 0) {    
      switch (ccurrentState) {                         // flag to indicate the servos' movement
        
        // first move the sorting door to the good gem position
        case SERVO1_FORWARD:                          
          positionServo1 += ((endAngleServo1 - startAngleServo1) / (1000.0 / interval)) * speedFactorServo1;
          if (positionServo1 >= endAngleServo1) {  
            positionServo1 = endAngleServo1;
            ccurrentState = SERVO2_FORWARD;
          }
          Bot.ToPosition("S1", degreesToDutyCycle(positionServo1));
          break;
        
        // open the flow control gate to let the gem through
        case SERVO2_FORWARD:
          positionServo2 += 1;
            if (positionServo2 >= endAngleServo2) {
            positionServo2 = endAngleServo2;
            ccurrentState = SERVO2_REVERSE;
          }
          Bot.ToPosition("S2", degreesToDutyCycle(positionServo2));
          break;
        
        // close the flow control gate to prevent more gems from coming through
        case SERVO2_REVERSE:
          positionServo2 -= 1;
          if (positionServo2 <= startAngleServo2) {
            positionServo2 = startAngleServo2;
            ccurrentState = SERVO1_REVERSE;
          }
          Bot.ToPosition("S2", degreesToDutyCycle(positionServo2));
          break;

        // reset the position of the flow contr
        case SERVO1_REVERSE:
          positionServo1 -= ((endAngleServo1 - startAngleServo1) / (1000.0 / interval)) * speedFactorServo1;
          if (positionServo1 <= startAngleServo1) {
            positionServo1 = startAngleServo1;
            ccurrentState = SERVO1_FORWARD;
          }
          Bot.ToPosition("S1", degreesToDutyCycle(positionServo1));
          break;

        // reset the sample flag to test next gem colour
        case SAMPLE_AGAIN:
          sample = false;
          break;
        }
    }

    // if the gem is bad
    else if (sortMode == 1) {
      
      switch (ccurrentState) {                         // flag to indicate the servos' movement

        // move the sorting door
        case SERVO2_FORWARD:
          positionServo2 += 1;
          if (positionServo2 >= endAngleServo2) {
            positionServo2 = endAngleServo2;
            ccurrentState = SERVO2_REVERSE;
          }
          Bot.ToPosition("S2", degreesToDutyCycle(positionServo2));
          break;
        
        // close the sorting door for bad gems
        case SERVO2_REVERSE:
          positionServo2 -= 1;
          if (positionServo2 <= startAngleServo2) {
            positionServo2 = startAngleServo2;
            ccurrentState = SERVO1_FORWARD;
          }
          Bot.ToPosition("S2", degreesToDutyCycle(positionServo2));
          break;

        // open the flow control door to let the gem out 
        case SERVO1_FORWARD:
          positionServo1 += ((endAngleServo1 - startAngleServo1) / (1000.0 / interval)) * speedFactorServo1;
          if (positionServo1 >= endAngleServo1) {
            positionServo1 = endAngleServo1;
            ccurrentState = SERVO2_REVERSE;
          }
          Bot.ToPosition("S1", degreesToDutyCycle(positionServo1));
          break;
        
        // close the flow control door to prevent extra gems through
        case SERVO1_REVERSE:
          positionServo1 -= ((endAngleServo1 - startAngleServo1) / (1000.0 / interval)) * speedFactorServo1;
          if (positionServo1 <= startAngleServo1) {
            positionServo1 = startAngleServo1;
            ccurrentState = SERVO1_FORWARD;
          }
          Bot.ToPosition("S1", degreesToDutyCycle(positionServo1));
          break;

        // reset the sample flag to test next gem colour
        case SAMPLE_AGAIN:
          sample = false;
          break;

      }
    }

    // if there is no gem sensed, reset the sampling flag
    else if(sortMode == 2) {
      Serial.printf("Do nothing for sorting\n");  // if there is no good or bad gem, sort nothing 
      sample = false;
    }
  }  
}

// function to return a dutyCyle that should be written using pwm to the servos
long degreesToDutyCycle(int deg) {
  const long MinimumDC = 400; // Duty Cycle for 0 degrees
  const long MaximumDC = 2100; // Duty Cycle for 180 degrees
  long dutyCycle = map(deg, 0, 180, MinimumDC, MaximumDC); // map from degrees to duty cycle
  return dutyCycle;
}

// Set colour of Smart LED depending on robot mode (and update brightness)
void Indicator() {
  SmartLEDs.setPixelColor(0, modeIndicator[robotModeIndex]);                  // set pixel colors to = mode 
  SmartLEDs.show();                                                           // send the updated pixel colors to the hardware
}