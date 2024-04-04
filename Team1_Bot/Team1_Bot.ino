/*

 MSE 2202 Team 1 Bot Drive System Code
 Language: Arduino
 Author: Mia Macdonald-Walden

 Rev 1  - Initial Version March 20 2024
 Rev 2  - Changed drive system path

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

// import relevant packages
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <MSE2202_Lib.h>
#include <NewPing.h>


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
#define MODE_BUTTON         0                                                  // GPIO0  pin 27 for Push Button 1
#define MOTOR_ENABLE_SWITCH 3                                                  // DIP Switch S1-1 pulls Digital pin D3 to ground when on, connected to pin 15 GPIO3 (J3)
#define SMART_LED           21                                                 // when DIP Switch S1-4 is on, Smart LED is connected to pin 23 GPIO21 (J21)
#define SMART_LED_COUNT     1                                                  // number of smart led's 
#define TRIGGER_PIN            13                                                 // GIPO pin for the ultrasonic sensor (trig)
#define ECHO_PIN            14                                                 // GIPO pin for the ultrasonic sensor (echo)
#define MAX_DISTANCE        30                                                  
#define PI                  3.1415926535897932384626433832795                  // const. value for PI

// Constants
const int cDisplayUpdate = 100;                                                // update interval for Smart LED in milliseconds
const int cPWMRes = 4;                                                         // bit resolution for PWM
const int cMinPWM = 150;                                                       // PWM value for minimum speed that turns motor
const int cMaxPWM = pow(2, cPWMRes) - 1;                                       // PWM value for maximum speed
const int cCountsRev = 1096;                                                   // encoder pulses per motor revolution
const float cRevDistance = 25.761;                                             // distance traversed by the bot for 1 wheel revolution
const float cDriveDistance = 200;                                                     // distance for the bot to travel forward/backward
const float cInitialDrive = 25;                                                 // distance for the bot to initially travel out

// adjustment variables and drive speed
const int cLeftAdjust = 0;                                                     // Amount to slow down left motor relative to right
const int cRightAdjust = 1;                                                    // Amount to slow down right motor relative to left

// Variables
boolean motorsEnabled = true;                                                  // motors enabled flag
boolean returning = false;                                                     // flag to indicate whether the ultrasonic sensor must wait
unsigned int turnNo = 0;                                                       // indicates number of turns that have been completed
unsigned char leftDriveSpeed = 255 - cLeftAdjust;                              // motor drive speed (0-255)
unsigned char rightDriveSpeed = 255 - cRightAdjust;                            // motor drive speed (0-255)
unsigned char leftTurnSpeed = 200 - cLeftAdjust;
unsigned char rightTurnSpeed = 200 - cRightAdjust;
unsigned int robotModeIndex = 0;                                               // state index for run mode
unsigned int driveIndex = 0;                                                   // state index for drive
unsigned int homeIndex = 0;                                                    // state index for return drive
unsigned long displayTime;                                                     // heartbeat LED update timer
unsigned long previousMicros;                                                  // last microsecond count
unsigned long currentMicros;                                                   // current microsecond count

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

// functions 
int getDistance();
void Indicator();

// Motor and encoder objects (classes defined in MSE2202_Lib)
Motion Bot = Motion();                                                         // Instance of Motion for motor control
Encoders LeftEncoder = Encoders();                                             // Instance of Encoders for left encoder data
Encoders RightEncoder = Encoders();                                            // Instance of Encoders for right encoder data

// ultrasonic object
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

void setup() {
  Serial.begin(115200);

   // Set up motors and encoders
   Bot.driveBegin("D1", LEFT_MOTOR_A, LEFT_MOTOR_B, RIGHT_MOTOR_A, RIGHT_MOTOR_B);  // set up motors as Drive 1
   LeftEncoder.Begin(ENCODER_LEFT_A, ENCODER_LEFT_B, &Bot.iLeftMotorRunning );      // set up left encoder
   RightEncoder.Begin(ENCODER_RIGHT_A, ENCODER_RIGHT_B, &Bot.iRightMotorRunning );  // set up right encoder

   // Set up SmartLED
   SmartLEDs.begin();                                                          // initialize smart LEDs object (REQUIRED)
   SmartLEDs.clear();                                                          // clear pixel
   SmartLEDs.setPixelColor(0,SmartLEDs.Color(0,0,0));                          // set pixel colors to 'off'
   SmartLEDs.show();                                                           // send the updated pixel colors to the hardware

   pinMode(MOTOR_ENABLE_SWITCH, INPUT_PULLUP);                                 // set up motor enable switch with internal pullup
   pinMode(MODE_BUTTON, INPUT_PULLUP);                                         // Set up mode pushbutton
}

void loop() {
  long pos[] = {0, 0};                                                        // current motor positions
   
  currentMicros = micros();                                                   // get current time in microseconds
  if((currentMicros - previousMicros) >= 1000){                               // enter if 1 millisecond has passed since last entry
    previousMicros = currentMicros;                                           // record current time in microseconds 

    // Mode pushbutton debounce and toggle
    if (!digitalRead(MODE_BUTTON)) {                                            // if pushbutton GPIO goes LOW (nominal push)
      robotModeIndex = 1;
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
        RightEncoder.clearEncoder();                                            // reset right encoder count
        driveIndex = 0;                                                         // set the drive index to 1 (first driving state)
        break;

      case 1: // drive                              
        if(motorsEnabled){
          switch(driveIndex){                                                   // drive mode control
            case 0: //Drive forward into square 
              Bot.Forward("D1", leftDriveSpeed, rightDriveSpeed);
              RightEncoder.getEncoderRawCount();
              if(RightEncoder.lRawEncoderCount >= cCountsRev * (cInitialDrive/cRevDistance)){
                RightEncoder.clearEncoder();
                Serial.println("Drive forward");
                driveIndex = 1;
              }
              break;

            case 1: // turn right
              Bot.Right("D1", leftTurnSpeed, rightTurnSpeed);
              RightEncoder.getEncoderRawCount();
              if(RightEncoder.lRawEncoderCount >= cCountsRev * ((PI/2)*7.4/cRevDistance)){
                RightEncoder.clearEncoder();
                Serial.println("right turn");
                driveIndex = 2;
              }
              break;

            case 2: // drive forward
              Bot.Forward("D1", leftDriveSpeed, rightDriveSpeed);
              RightEncoder.getEncoderRawCount();
              if(RightEncoder.lRawEncoderCount >= cCountsRev * ((cDriveDistance/2)/cRevDistance)){
                RightEncoder.clearEncoder();
                Serial.println("Half drive forward");
                driveIndex = 3;
              }
              break;

            case 3: // turn left
              Bot.Left("D1", leftTurnSpeed, rightTurnSpeed);
              RightEncoder.getEncoderRawCount();
              if(RightEncoder.lRawEncoderCount <= cCountsRev * ((PI/2)*3.7*-1/cRevDistance)){
                RightEncoder.clearEncoder();
                Serial.println("Turn left");
                driveIndex = 4;
                turnNo++;
                if(turnNo == 4){
                  driveIndex = 2;
                }
                if(turnNo == 5){
                  driveIndex = 4;
                }
              }
              break;

            case 4: // sweep forward
              Bot.Forward("D1", leftDriveSpeed, rightDriveSpeed);
              RightEncoder.getEncoderRawCount();
              if(RightEncoder.lRawEncoderCount >= cCountsRev * (cDriveDistance/cRevDistance)){
                RightEncoder.clearEncoder();
                Serial.println("Forward");
                driveIndex = 3;
                if(turnNo == 5){
                  robotModeIndex = 3;
                }
              }
              break; 
          }
        }
        break;

      case 2: // operate pick up
        // code for pick up (not yet completed)
        Serial.println("pick up");                                                    
        robotModeIndex = 3;                                                      // switch back to the drive mode
        break;

      case 3: // navigate to home base
        Bot.Reverse("D1", leftDriveSpeed, rightDriveSpeed);
        /*
        int currentDist = getDistance();
        if(currentDist == 0){
          Bot.Stop("D1");
          robotModeIndex = 0;
        }
        */
        RightEncoder.getEncoderRawCount();
        if(RightEncoder.lRawEncoderCount >= cCountsRev * (cDriveDistance + cInitialDrive)/cRevDistance){
          RightEncoder.clearEncoder();
          robotModeIndex = 0;
        }
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

// sorting system operation
void Sorting() {
  // code to operate sorting and sorting servos (not yet completed)
}

int getDistance() {
  int d = 0;
  for(int i = 1; i <= 8; i++)
  {
    d = (d + sonar.ping_cm())/i;
  }
  return d;
}

// Set colour of Smart LED depending on robot mode (and update brightness)
void Indicator() {
  SmartLEDs.setPixelColor(0, modeIndicator[robotModeIndex]);                  // set pixel colors to = mode 
  SmartLEDs.show();                                                           // send the updated pixel colors to the hardware
}