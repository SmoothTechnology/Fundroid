#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// ProtoTypes
boolean IsAngleAcceptable(int a);
int GetSpeedValue(int curTicks, int destTicks);
void goToAngle(int a);
void GoToPoint(int point);
void rRising();
void lRising();
void setWheelSpeeds();
int GetForwardSpeedValue(int curTicks, int destTicks);
void goToTarget();
void newTarget(int x);
int GetDesiredWheelTicks(float deltaAngle);
void calcAngleCoordinates();
void printInfo();
void infoToProcessing();

#define COORDINATOR_PORT Serial1
enum MotorControls{RotateAbsolute = 0, MoveToPosition = 1, Stop = 2, Start = 3};
enum MotorErrors{MotionComplete = 0, RotationFailure = 1, MoveToPositionFailure = 2};

int motionToDo = 0;
int curPosition = 0;
boolean isInitialized = false;

#define BNO055_SAMPLERATE_DELAY_MS (50)

Adafruit_BNO055 bno = Adafruit_BNO055();

void SetCalibrationData(adafruit_bno055_offsets_t &calibData)
{
  calibData.accel_offset_x = 20;
  calibData.accel_offset_y = 65454;
  calibData.accel_offset_z = 0;

  calibData.gyro_offset_x = 0;
  calibData.gyro_offset_y = 65534;
  calibData.gyro_offset_z = 65535;

  calibData.mag_offset_x = 65354;
  calibData.mag_offset_y = 221;
  calibData.mag_offset_z = 65323;

  calibData.accel_radius = 1000;
  calibData.mag_radius = 839;
}

void WaitForCalibrationComplete()
{
  int numberConsecutiveReads = 3;
  int numReads = 0;
  /* Display calibration status for each sensor. */
  uint8_t system, gyro, accel, mag = 0;

  while(numReads != numberConsecutiveReads)
  {
    bno.getCalibration(&system, &gyro, &accel, &mag);
    Serial.print("Continue when Mag == 3... Currently Mag=");
    Serial.println(mag, DEC);
    delay(BNO055_SAMPLERATE_DELAY_MS);

    if(mag == 3)
    {
      numReads++;
    }
    else
    {
      numReads = 0;
    }
  }
  
}

void StartBNO()
{
  if(!bno.begin(Adafruit_BNO055::OPERATION_MODE_COMPASS))
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  adafruit_bno055_offsets_t calibrationData;
  SetCalibrationData(calibrationData);
  bno.setSensorOffsets(calibrationData);

  bno.setExtCrystalUse(true);

  WaitForCalibrationComplete();
}

long lastBNORead = 0;
long nextBNORead = BNO055_SAMPLERATE_DELAY_MS;
float curBNOHeading = 0;
float lastBNOHeading = 0;
void ReadBNO()
{
  if(millis() - lastBNORead > nextBNORead)
  {
    // Possible vector values can be:
    // - VECTOR_ACCELEROMETER - m/s^2
    // - VECTOR_MAGNETOMETER  - uT
    // - VECTOR_GYROSCOPE     - rad/s
    // - VECTOR_EULER         - degrees
    // - VECTOR_LINEARACCEL   - m/s^2
    // - VECTOR_GRAVITY       - m/s^2
    if(!isInitialized)
    {
      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

      /* Display the floating point data */
      Serial.print("X: ");
      Serial.print(euler.x());
      Serial.print(" Y: ");
      Serial.print(euler.y());
      Serial.print(" Z: ");
      Serial.print(euler.z());
      Serial.println("\t\t");

      curBNOHeading = euler.x();
    }
    else
    {
      sensors_event_t event;
      bno.getEvent(&event);
      Serial.print(F("Orientation: "));
      Serial.print((float)event.orientation.x);
      Serial.print(F(" "));
      Serial.print((float)event.orientation.y);
      Serial.print(F(" "));
      Serial.print((float)event.orientation.z);
      Serial.println(F(""));

      curBNOHeading = event.orientation.x;
    }

    lastBNORead = millis();
  }
}


/*
  Wheels are 96 steps per rotation 
  Now there are 192 since we have 6 pulses per internal motor revolution
  Wheel Diameter: 253mm
  Wheel Circumference: 796mm, maybe different though
  distance per step: 8.29mm  
  Wheelbase = 502mm
*/
int inByte = 0;         // incoming serial byte

const int dirPinR = 8;
const int spdPinR = 9;
const int dirPinL = 10;
const int spdPinL = 11;
const int resetButton = 12;
const int led = 13;
int topSpeed = 70;
int bottomSpeed = 30;

boolean goingStraight = 0;

int rWheelTicks = 0;
int lWheelTicks = 0;
int rWheelChange = 0;         //for calculating change since last sample
int lWheelChange = 0;

int rWheelSpeed = 0;
int lWheelSpeed = 0;

float rWheelDist = 0;
float lWheelDist = 0;
float distCenter = 0;

float botAngle = 0;
float angleChange = 0;

float botX = 0;
float botY = 0;

float changeX = 0;
float changeY = 0;

// 32:1 Gear Ratio
float wheelBase = 494.5;
float wheelCircumference = 784;     //based on measuring w a wire (was originally 796)
float numOfSteps = 192;
float stepDist = wheelCircumference/numOfSteps;

int rHitTarget = 0;
int lHitTarget = 0;

const byte interruptPin1 = 2;
const byte interruptPin2 = 3;
int rTarget = -10;
int lTarget = -10;

boolean enterNudgeSequence = false;
boolean inMotion = false;
boolean targetSet = false;
float CorrectAngle = 0.0;

// Return Success = 0, Failure = 1
int ParseCommand(char ControlByte, String ControlArgument)
{
  ControlByte = ControlByte - '0';

  if(ControlByte == RotateAbsolute)
  {
    motionToDo = ControlByte;
    curPosition = ControlArgument.toInt();
    Serial.print("GO TO ANGLE: ");
    Serial.println(curPosition);
    inMotion = true;
    goToAngle(curPosition);

#ifdef DEBUG
    Serial.print("Found rotate command with value of ");
    Serial.println(curMotion.position);
#endif
  }
  else if(ControlByte == MoveToPosition)
  {
    motionToDo = ControlByte;
    curPosition = ControlArgument.toInt();
    Serial.print("GO TO POSITION: ");
    Serial.println(curPosition);
    inMotion = true;
    GoToPoint(curPosition);

    #ifdef DEBUG
    Serial.print("Found Move command with value of ");
    Serial.println(curMotion.position);
    #endif
  }
  else if(ControlByte == Stop)
  {
    StopBot();
    inMotion = false;
    #ifdef DEBUG
    Serial.println("Disarmed");
    #endif
  }
  else if(ControlByte == Start)
  {
    #ifdef DEBUG
    Serial.println("Armed");
    #endif
  }
  else
  {
    #ifdef DEBUG
    Serial.println("Command Not Found");
    #endif

    return 1; // Command Not Found
  }

  return 0;
}

// Return Success = 0, Failure = 1
int CheckForCommands()
{
  enum ReadStates{ReadCommand, ReadComma, ReadArgument, end};

  char ControlByte;
  String ControlArgument;

  ReadStates state = ReadCommand;
  if(COORDINATOR_PORT.available() > 4)
  {
    while(COORDINATOR_PORT.available())
    {
      if(state == ReadCommand)
      {
        ControlByte = COORDINATOR_PORT.read();
        state = ReadComma;
      }
      else if(state == ReadComma)
      {
        char comma = COORDINATOR_PORT.read();

        if(comma != ',')
        {
          return 1;
        }

        state = ReadArgument;
      }
      else if(state == ReadArgument)
      {
        ControlArgument = COORDINATOR_PORT.readStringUntil('&');
        state = end;
      }
      else
      {
        break;
      }
    }

    return ParseCommand(ControlByte, ControlArgument);
  }

  return 0;
}

boolean IsInitializeAngleClose()
{
  if(curBNOHeading > 359.0 || curBNOHeading < 1.0)
    return true;

  return false;
}

void StopBot()
{
    digitalWrite(dirPinR, LOW);               //FWD
    analogWrite(spdPinR, 0);
    digitalWrite(dirPinL, HIGH);               //FWD
    analogWrite(spdPinL, 0);
}

void RotateBotCCW()
{
  digitalWrite(dirPinR, LOW);               //FWD
  analogWrite(spdPinR, 40);
  digitalWrite(dirPinL, HIGH);               //FWD
  analogWrite(spdPinL, 40);
}

void RotateBotCW()
{
  digitalWrite(dirPinR, HIGH);               //FWD
  analogWrite(spdPinR, 40);
  digitalWrite(dirPinL, LOW);               //FWD
  analogWrite(spdPinL, 40);
}

void NudgeToZero()
{
  while(!IsInitializeAngleClose())
  {
    Serial.println("NUDGE");
    if(curBNOHeading < 180)
    {
      RotateBotCCW();
      DelayAndReadBNO(100);
      StopBot();
    }
    else 
    { 
      RotateBotCW();
      DelayAndReadBNO(100);
      StopBot();
    }

    DelayAndReadBNO(2000);
  }
  
}

void InitializeBot()
{
  boolean lessThan180 = false;
  if(curBNOHeading < 180)
  {
    RotateBotCCW();
    lessThan180 = true;
  }
  else 
  {
    RotateBotCW();
    lessThan180 = false;
  }

  while(!isInitialized)
  {
    ReadBNO();

    if(lessThan180 && curBNOHeading > 180)
    {
      StopBot();
      NudgeToZero();
    }
    else if(!lessThan180 && curBNOHeading < 180)
    {
      StopBot();
      NudgeToZero();
    }

    if(IsInitializeAngleClose())
    {
      StopBot();
      botAngle = 0;
      isInitialized = true;
      bno.begin();
      Serial.println("Initialization Complete. Continuing in 2 seconds");
      delay(2000);
    }
  }
}

void DelayAndReadBNO(long delayTime)
{
  Serial.print("Delay At ");
  Serial.print(delayTime);
  long curTime = millis();
  while(millis() < curTime + delayTime)
  {
    ReadBNO();
  }
}

void setup() {

  Serial1.begin(9600);
  Serial.begin(9600);
 
  StartBNO();
 
  //Motor Controls
  pinMode (dirPinR, OUTPUT);
  pinMode (spdPinR, OUTPUT);
  analogWrite(spdPinR, 0);
  pinMode (dirPinL, OUTPUT);
  pinMode (spdPinL, OUTPUT);
  analogWrite(spdPinL, 0);
  digitalWrite(dirPinR, HIGH);
  digitalWrite(dirPinL, HIGH);

  //Encoder Read Pins
  pinMode(2, INPUT_PULLUP); //  Opto Reader 1    (RIGHT)
  pinMode(4, INPUT);        //  Opto Reader 2    (RIGHT)
  pinMode(3, INPUT_PULLUP); //  Opto Reader 3    (LEFT)
  pinMode(5, INPUT);        //  Opto Reader 4    (LEFT)
  attachInterrupt(digitalPinToInterrupt(interruptPin1), rRising, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPin2), lRising, CHANGE);

  pinMode(resetButton, INPUT_PULLUP);
  pinMode(led, OUTPUT);

  //  Serial.println("press grey button to start...");
  //
  //while(digitalRead(13) == HIGH){
  //  delay(100);
  //}

//  Serial.println("starting");
  digitalWrite(led, HIGH);
  DelayAndReadBNO(500);
  digitalWrite(led, LOW);
  DelayAndReadBNO(5000);
}

int curMove = 0;

long heartBeatInterval = 1000;
long lastTime = 0;
void loop() {
//setWheelSpeeds();
//printInfo();            //nice looking display w labels
//infoToProcessing();       //CSV for processing

  ReadBNO();

  if(!isInitialized)
  {
    InitializeBot();
  }

  CheckForCommands();

  if(motionToDo == RotateAbsolute && inMotion)
  {
    goToAngle(curPosition);
  }
  else if(motionToDo == MoveToPosition && inMotion)
  {
    GoToPoint(curPosition);
  }

  calcAngleCoordinates();

  if(!inMotion && isInitialized && (millis() - lastTime) > heartBeatInterval)
  {
    lastTime = millis();
    targetSet = false;
    Serial.println("Motion1 Complete");
    COORDINATOR_PORT.print('0');
  }

}

boolean IsAnglePreNudgeAcceptable(int a)
{
  if(a+4 > botAngle*57.3 && a-4 < botAngle*57.3)
  {
    return true;
  }

  return false;
}

int marginOfError = 1;
boolean IsAngleAcceptable(int a)
{
  if(a+1.5 > botAngle*57.3 && a-1.5 < botAngle*57.3)
  {
    return true;
  }

  return false;
}

float kP = 0.01;
int GetSpeedValue(int curTicks, int destTicks)
{

  int error = destTicks - curTicks;
  int spd = error*kP;
  if(spd < 0)
    spd = -1*spd;

  if(spd < 30)
    spd = 30;
  else if(spd > 50)
    spd = 50;

  return spd;
}

boolean ShouldRotateCCW(int a)
{
  float curBotAngleD = botAngle*57.3;

  boolean CCWRotate = true;

  float diffAngle = 0.0;

  if(curBotAngleD > a)
  {
    CCWRotate = true;
    diffAngle = curBotAngleD - a;
  }
  else
  {
    CCWRotate = false;
    diffAngle = a - curBotAngleD;
  }
  
  if(diffAngle > 180)
  {
    CCWRotate = !CCWRotate;
  }

  return CCWRotate;
}

void nudgeToAngle(int a)
{
  boolean CCWRotate = ShouldRotateCCW(a);

  float kp = 2;
  float botAngleDegree = botAngle*57.3;
  float diff = abs(botAngle-a);


  if(CCWRotate)
  {
    RotateBotCCW();
    DelayAndReadBNO(100);
    // DelayAndReadBNO(kp*diff);
    StopBot();
  }
  else
  {
    RotateBotCW();
    DelayAndReadBNO(100);
    //DelayAndReadBNO(kp*diff);
    StopBot();
  }

  DelayAndReadBNO(1500);

}

void goToAngle(int a){ 

  CorrectAngle = (float)a;

  Serial.println(botAngle*57.3);

  if(IsAnglePreNudgeAcceptable(a) && !enterNudgeSequence)
  {
    StopBot();
    enterNudgeSequence = true;
    DelayAndReadBNO(1000);
  }
  else if(enterNudgeSequence && IsAngleAcceptable(a))
  {
    inMotion = false;
    enterNudgeSequence = false;
  }
  else if(enterNudgeSequence)
  {
    nudgeToAngle(a);
  }
  else
  {
    enterNudgeSequence = false;

    if(ShouldRotateCCW(a))
    {
      int wheelTicks = GetDesiredWheelTicks(a);
      Serial.println(wheelTicks);

      int spd1 = GetSpeedValue(rWheelTicks ,wheelTicks);
      int spd2 = GetSpeedValue(lWheelTicks, wheelTicks);
      Serial.print("spd1 ");
      Serial.print(spd1);
      Serial.print(" spd2 ");
      Serial.println(spd2);

      digitalWrite(dirPinR, LOW);               //FWD
      analogWrite(spdPinR, GetSpeedValue(rWheelTicks ,wheelTicks));
      digitalWrite(dirPinL, HIGH);               //FWD
      analogWrite(spdPinL, GetSpeedValue(lWheelTicks ,wheelTicks));
    }
    else
    {
      int wheelTicks = GetDesiredWheelTicks(a);
      Serial.println(wheelTicks);

      int spd1 = GetSpeedValue(rWheelTicks ,wheelTicks);
      int spd2 = GetSpeedValue(lWheelTicks, wheelTicks);
      Serial.print("spd1 ");
      Serial.print(spd1);
      Serial.print(" spd2 ");
      Serial.println(spd2);

      digitalWrite(dirPinR, HIGH);               //FWD
      analogWrite(spdPinR, GetSpeedValue(rWheelTicks ,wheelTicks));
      digitalWrite(dirPinL, LOW);               //FWD
      analogWrite(spdPinL, GetSpeedValue(lWheelTicks ,wheelTicks));
    }
    
    inMotion = true;
  }
}

void GoToPointMetric(float meter)
{
  int nextPoint = meter*240;

  GoToPoint(nextPoint);
}

void GoToPoint(int point)
{
  if(!targetSet)
  {
    rTarget = point+rWheelTicks;
    lTarget = point+lWheelTicks;
    targetSet = true;
  }
  goToTarget();
}


void rRising() {
  if (digitalRead(2) == HIGH){
  if (digitalRead(4) == HIGH) {
    rWheelTicks--;
    rWheelChange--;
  }
  else {
    rWheelTicks++;
    rWheelChange++;
  }
  }
  else{
     if (digitalRead(4) == HIGH) {
    rWheelTicks++;
    rWheelChange++;
  }
  else {
    rWheelTicks--;
    rWheelChange--;
  }
  }
}

void lRising() {
  if (digitalRead(3) == HIGH){
   if (digitalRead(5) == HIGH) {
    lWheelTicks--;
    lWheelChange--;
   }
   else {
    lWheelTicks++;
    lWheelChange++;
    }
   }
  else{
   if (digitalRead(5) == HIGH) {
    lWheelTicks++;
    lWheelChange++;
   }
   else {
    lWheelTicks--;
    lWheelChange--;
    }
  }
}

void setWheelSpeeds(){
  rWheelSpeed = map(abs(rTarget - rWheelTicks), 100, 0, topSpeed, bottomSpeed);       // these need to change (the mapping from 100-0 is not correct
  lWheelSpeed = map(abs(lTarget - lWheelTicks), 100, 0, topSpeed, bottomSpeed);
  rWheelSpeed = constrain(rWheelSpeed, bottomSpeed, topSpeed);
  lWheelSpeed = constrain(lWheelSpeed, bottomSpeed, topSpeed);
}


float fkP = 0.7;
int GetForwardSpeedValue(int curTicks, int destTicks)
{

  int error = destTicks - curTicks;
  int spd = error*fkP;
  if(spd < 0)
    spd = -1*spd;

  if(spd < 30)
    spd = 30;
  else if(spd > 100)
    spd = 100;

  return spd;
}

void goToTarget(){
  inMotion = true;

  int rWheelSpeed = GetForwardSpeedValue(rWheelTicks, rTarget);

  int rDiff = abs(rTarget - rWheelTicks);
  int lDiff = abs(lTarget - lWheelTicks);

  float ikp = 0.5;
  int lrDiff = rDiff - lDiff;
  if(lrDiff > 0) // Add to r speed because r is further away
  {
    rWheelSpeed += ikp*lrDiff;
  }
  

  if (rWheelTicks <rTarget) {
    rHitTarget = 0;
    digitalWrite(dirPinR, LOW);               //FWD
    analogWrite(spdPinR, rWheelSpeed);
  }
  else if (rWheelTicks > rTarget) {
    rHitTarget = 0;
    digitalWrite(dirPinR, HIGH);              //REV
    analogWrite(spdPinR, rWheelSpeed);
  }
  else {
    analogWrite(spdPinR, 0);
    rHitTarget = 1;
  }

  int lWheelSpeed = GetForwardSpeedValue(lWheelTicks, lTarget);
  if(lrDiff < 0)
  {
    lWheelSpeed += ikp*abs(lrDiff);
  }

  if (lWheelTicks < lTarget) {
    lHitTarget = 0;
    digitalWrite(dirPinL, LOW);
    analogWrite(spdPinL, lWheelSpeed);
  }
  else if (lWheelTicks > lTarget) {
    lHitTarget = 0;
    digitalWrite(dirPinL, HIGH);
    analogWrite(spdPinL, lWheelSpeed);
  }
  else {
    analogWrite(spdPinL, 0);
    lHitTarget = 1;
  }

  if(lHitTarget && rHitTarget)
  {
    inMotion = false;
  }

  Serial.print("RWheelSpeed :");
  Serial.print(rWheelSpeed);
  Serial.print(" LWheelSPeed :");
  Serial.print(lWheelSpeed);
  Serial.print(" LWheelTicks :");
  Serial.print(lWheelTicks);
  Serial.print(" rWheelTicks :");
  Serial.print(rWheelTicks);
  Serial.print(" Diff :");
  Serial.println(lrDiff);
}

void newTarget(int x){
  if (lHitTarget + rHitTarget == 2) {
    rTarget = random(-x, x);
    lTarget = random(-x, x);
  //  Serial.println(lTarget);
    rHitTarget = 0;
    lHitTarget = 0;
    delay(500);
  }
}

int GetDesiredWheelTicks(float deltaAngle)
{
  deltaAngle = botAngle*57.3 - deltaAngle;
  
  int desiredWheelTicks = ((deltaAngle*wheelBase)/2) / stepDist;

  return desiredWheelTicks;
}

void calcAngleCoordinates() {

  rWheelDist = rWheelChange * stepDist;
  lWheelDist = lWheelChange * stepDist;
  angleChange = (rWheelDist - lWheelDist) / wheelBase;
  
  botAngle = botAngle + angleChange;
  distCenter = (rWheelDist + lWheelDist)/2; 
  botX = botX + distCenter*cos(botAngle);
  botY = botY + distCenter*sin(botAngle);

  if(curBNOHeading != lastBNOHeading)
  {
    lastBNOHeading = curBNOHeading;
    botAngle = curBNOHeading/57.3;
  }
  
  rWheelChange = 0;
  lWheelChange = 0;
  angleChange = 0;

}

void printInfo() {
  Serial.print("     lTarget: ");
  Serial.print(lTarget);
    Serial.print("  rTarget: ");
  Serial.print(rTarget);
  Serial.print("   lWhlChange: ");
  Serial.print(lWheelChange);

  Serial.print("   rWhlChange: ");
  Serial.print(rWheelChange);

  Serial.print("  botAngle(degrees): ");
  Serial.print((botAngle*57.3));
  Serial.print("    botX: ");
  Serial.print(botX/100);
  Serial.print("    botY: ");
  Serial.print(botY/100);
  
  Serial.println();
}

void infoToProcessing(){

    Serial.print(botX/100, DEC);
    Serial.print(",");
    Serial.print(botY/100, DEC);
    Serial.print(",");
    Serial.println((botAngle*57.3), DEC);
 
}

