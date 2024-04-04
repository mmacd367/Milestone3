// Include the follwing files into the program
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_TCS34725.h"
#include <MSE2202_Lib.h>
#include <NewPing.h>

//Encoder structure
struct Encoder{
  const int chanA; //gpio pin for encoder channel A
  const int chanB; //gpio pin for encoder channel B
  long pos; // current encoder position
};

// Servo pins
#define SERVO3 40
#define SERVO4 41
#define SERVO5 1

// DC motor related pins
#define LEFT_MOTOR_A 35
#define LEFT_MOTOR_B 36
#define RIGHT_MOTOR_A 37
#define RIGHT_MOTOR_B 38
#define ENCODER_LEFT_A 15
#define ENCODER_LEFT_B 16
#define ENCODER_RIGHT_A 11
#define ENCODER_RIGHT_B 12
#define PI 3.1415
#define TRIGGER_PIN  13
#define ECHO_PIN     14
#define MAX_DISTANCE 30

#define PWMCHAN_SERVO3 4
#define PWMCHAN_SERVO4 5
#define PWMCHAN_SERVO5 6


// constants for dc motors

const int cPWMRes = 4; // bit resolution for pwm
const int cMinPWM = 150; // pwm value for minimum speed that turns motor
const int cMaxPWM = pow(2, cPWMRes) - 1; // max pwm value
const int cCountsRev = 1096; // encoder pulses per motor rev
const float cRevDistance = 25.761; // distance covered by 1 wheel revolution
const float cDriveDistance = 200; // distance for bot to travel forward/backward
const float cInitialDrive = 25; // distance for the bot to initially travel out

// adjustment variables for drive speed
const int cLeftAdjust = 0; // amount to slow down left motor relative to the right
const int cRightAdjust = 1;// amount to slow down right motor relative to the left

//more variables
bool motorsEnabled = true; // motors enabled flag
bool returning = false; // flag to indicate if ultrasonic sensor must wait
unsigned int turnNo = 0; // number of completed turns
unsigned char leftDriveSpeed = 255 - cLeftAdjust; // left speed adjustment
unsigned char rightDriveSpeed = 255 - cRightAdjust; // right speed adjustment
unsigned char leftTurnSpeed = 200 - cLeftAdjust; // left turn speed adjustment
unsigned char rightTurnSpeed = 200 - cRightAdjust; // right speed adjustment
unsigned int robotModeIndex = 0; // runmode state
unsigned int driveIndex = 0; //state index for drive
unsigned int homeIndex = 0; //state index for return drive
int Pickupflag = 1; // flag to switch between cases of pickup


// start and end angles for each servo
int startAngleServo3 = 0;  // Initial angle for servo 3
int endAngleServo3 = 90;   // Final angle for servo 3
int startAngleServo4 = 0; // Initial angle for servo 4
int endAngleServo4 = 90;  // Final angle for servo 4
int startAngleServo5 = 0; // Initial angle for servo 5
int endAngleServo5 = 180; // Final angle for servo 5

unsigned long previousMillis = 0; // previous millisecond count
unsigned long interval = 20; // Time interval between servo position updates (in milliseconds)

float positionServo3 = startAngleServo3; // Current position of servo 3
float positionServo4 = startAngleServo4; // Current position of servo 4
float positionServo5 = endAngleServo5; // Current position of servo 5

int speedFactorServo3 = 1; // Speed factor for servo 3
int speedFactorServo4 = 1; // Speed factor for servo 4
int speedFactorServo5 = 1; // Speed factor for servo 5


// function declarations
int degreesToDutyCycle(int deg);
void Pickup();
int getDistance();

Motion Bot = Motion();
Encoders LeftEncoder = Encoders();                                             // Instance of Encoders for left encoder data
Encoders RightEncoder = Encoders();                                            // Instance of Encoders for right encoder data

// ultrasonic object
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

void setup() {

  // config serial comms
  Serial.begin(115200);

  // setup motors and encoders
   Bot.driveBegin("D1", LEFT_MOTOR_A, LEFT_MOTOR_B, RIGHT_MOTOR_A, RIGHT_MOTOR_B);  // set up motors as Drive 1
   LeftEncoder.Begin(ENCODER_LEFT_A, ENCODER_LEFT_B, &Bot.iLeftMotorRunning );      // set up left encoder
   RightEncoder.Begin(ENCODER_RIGHT_A, ENCODER_RIGHT_B, &Bot.iRightMotorRunning );  // set up right encoder

  // set up the servo pins as outputs
  pinMode(SERVO3, OUTPUT);
  pinMode(SERVO4, OUTPUT);
  pinMode(SERVO5, OUTPUT);


  // set up LED channels for each servo
  ledcSetup(PWMCHAN_SERVO3, 40, 14); // pwm channel, frequency and bit resolution
  ledcSetup(PWMCHAN_SERVO4, 40, 14); // pwm channel, frequency and bit resolution
  ledcSetup(PWMCHAN_SERVO5, 40, 14); // pwm channel, frequency and bit resolution

  // atach the LED channels for each servo to different GPIO pins
  ledcAttachPin(SERVO3, PWMCHAN_SERVO3);
  ledcAttachPin(SERVO4, PWMCHAN_SERVO4);
  ledcAttachPin(SERVO5, PWMCHAN_SERVO5);

  // write a loop to intialize the claw to open
  while(positionServo5 > startAngleServo5){
    positionServo5 -= 1;
    ledcWrite(PWMCHAN_SERVO5, degreesToDutyCycle(positionServo5));
  }

}

void loop() {
  long pos[] = {0, 0};                                             // current motor positions
  unsigned long currentMillis = millis();
  unsigned long elapsedTime = currentMillis - previousMillis;
  switch (robotModeIndex){
    case 0: // robot stopped
      Bot.Stop("D1");                                                         // Stop the wheels
      RightEncoder.clearEncoder();                                            // reset right encoder count
      robotModeIndex = 1;                                                         // set the drive index to 1 (first driving state)
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
                  robotModeIndex = 2;
                }
              }
              break; 
          }
        }
        break;
      

    case 2: // Pickup
      if (elapsedTime >= interval){
        Pickup();
        previousMillis = currentMillis;
        }
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
  // Update servo positions based on elapsed time and speed factors
}

// Functions
// pickup the gems
void Pickup(){
  switch (Pickupflag) {
    case 1:
      positionServo5 += (endAngleServo5 - startAngleServo5) / (1000.0 / interval) / speedFactorServo5 ;
      if(positionServo5 >= endAngleServo5){
        Serial.println("test2");
        positionServo5 = endAngleServo5;
        Pickupflag = 2;
      }
      ledcWrite(PWMCHAN_SERVO5, degreesToDutyCycle(positionServo5));
      Serial.printf("%f\n", positionServo5);
      Serial.println("test if servo 5 is written 2 close ");
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
          robotModeIndex = 3;
      }
      ledcWrite(PWMCHAN_SERVO3, degreesToDutyCycle(positionServo3));
      ledcWrite(PWMCHAN_SERVO4, degreesToDutyCycle(positionServo4));
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