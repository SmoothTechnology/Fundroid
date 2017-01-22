/*------------------------------------------------------------------------------

  LIDARLite Arduino Library
  GetDistanceI2c

  This example shows how to initialize, configure, and read distance from a
  LIDAR-Lite connected over the I2C interface.

  Connections:
  LIDAR-Lite 5 Vdc (red) to Arduino 5v
  LIDAR-Lite I2C SCL (green) to Arduino SCL
  LIDAR-Lite I2C SDA (blue) to Arduino SDA
  LIDAR-Lite Ground (black) to Arduino GND
  
  (Capacitor recommended to mitigate inrush current when device is enabled)
  680uF capacitor (+) to Arduino 5v
  680uF capacitor (-) to Arduino GND

  See the Operation Manual for wiring diagrams and more information:
  http://static.garmin.com/pumac/LIDAR_Lite_v3_Operation_Manual_and_Technical_Specifications.pdf

------------------------------------------------------------------------------*/
#include <Bounce.h>
#include <Wire.h>
#include <LIDARLite.h>
#include <math.h>

#define MOTORCOMMPORT Serial1
#define BTN_PIN   6
#define LED_PIN   17

#define REAL double
#define MAX_READS 600

inline static REAL sqr(REAL x) {
    return x*x;
}

int linreg(int n, REAL x[], REAL y[], REAL* m, REAL* b, REAL* r)
{
    REAL   sumx = 0.0;                        /* sum of x                      */
    REAL   sumx2 = 0.0;                       /* sum of x**2                   */
    REAL   sumxy = 0.0;                       /* sum of x * y                  */
    REAL   sumy = 0.0;                        /* sum of y                      */
    REAL   sumy2 = 0.0;                       /* sum of y**2                   */

   for (int i=0;i<n;i++)   
      { 
      sumx  += x[i];       
      sumx2 += sqr(x[i]);  
      sumxy += x[i] * y[i];
      sumy  += y[i];      
      sumy2 += sqr(y[i]); 
      } 

   REAL denom = (n * sumx2 - sqr(sumx));
   if (denom == 0) {
       // singular matrix. can't solve the problem.
       *m = 0;
       *b = 0;
       if (r) *r = 0;
       return 1;
   }

   *m = (n * sumxy  -  sumx * sumy) / denom;
   *b = (sumy * sumx2  -  sumx * sumxy) / denom;
   if (r!=NULL) {
      *r = (sumxy - sumx * sumy / n) /          /* compute correlation coeff     */
            sqrt((sumx2 - sqr(sumx)/n) *
            (sumy2 - sqr(sumy)/n));
   }

   return 0; 
}

LIDARLite myLidarLite;

int dirPin = 2;
int stepPin = 3;
int stepTime = 10;
int curStep = 0;

int optoPin = 11;
double angleOffset = 0;

double sweepTo = 0.0;
double sweepFrom = 0.0;

int num_Readings = 0;

boolean FoundZero = false;
boolean SweepDone = false;
boolean SweepError = false;

#define staticDataSet 600
#define StepsPerRotation 400

struct LidarRead{
  double angle;
  int reading;
};

int curDataPoints = 0;
LidarRead dataPoints[staticDataSet];

double GetAngleFromStep(int step)
{
  double floatStep = step;
  return 360.0*(floatStep/StepsPerRotation) + angleOffset;
}

/////////////////////////////////////////////
//////// FUNNIE MOTOR FEET CONTROLS /////////
int stepsPerMeter = 240;
void MoveMotorForward(float meters)
{
  int stepsToMove = meters*stepsPerMeter;

  String First = "1,";
  String last = "&";
  String MoveForward = First + stepsToMove + last;

  MOTORCOMMPORT.println(MoveForward);
  Serial.print("Sending: ");
  Serial.println(MoveForward);
}

int curSystemAngle = 0;
void MoveMotorToAngle(int angle)
{
  String First = "0,";
  String last = "&";
  String MoveAngle = First + angle + last;

  MOTORCOMMPORT.println(MoveAngle);
  Serial.print("Sending: ");
  Serial.println(MoveAngle);

  curSystemAngle = angle;
}

boolean CheckForMotionComplete()
{
  if(MOTORCOMMPORT.available())
  {
    if(MOTORCOMMPORT.read() == '0')
    {
      return true;
    }
  }

  return false;
}

void StopMotor()
{
  String theCmd = "2,0&";

  MOTORCOMMPORT.println(theCmd);
  Serial.print("Sending: ");
  Serial.println(theCmd);
}

void FLUSHMOTORBUFFER()
{
  while(MOTORCOMMPORT.available())
  {
    MOTORCOMMPORT.read();
  }
}
////////////////////////////////////////////
////////////////////////////////////////////

void setup()
{
  delay(1000);
  
  MOTORCOMMPORT.begin(9600);
  Serial.begin(115200); // Initialize serial connection to display distance readings

  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(BTN_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(BTN_PIN, HIGH);
  digitalWrite(LED_PIN, LOW);
  
  digitalWrite(dirPin, 0);
  digitalWrite(stepPin, 0);
  
  pinMode(optoPin, INPUT);
  attachInterrupt(optoPin, fullRotation, FALLING); // interrrupt 1 is data ready


  
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
//  pinMode(14, OUTPUT);
//  digitalWrite(14, HIGH);
  
  /*
    begin(int configuration, bool fasti2c, char lidarliteAddress)

    Starts the sensor and I2C.

    Parameters
    ----------------------------------------------------------------------------
    configuration: Default 0. Selects one of several preset configurations.
    fasti2c: Default 100 kHz. I2C base frequency.
      If true I2C frequency is set to 400kHz.
    lidarliteAddress: Default 0x62. Fill in new address here if changed. See
      operating manual for instructions.
  */
  
  myLidarLite.begin(0, true); // Set configuration to default and I2C to 400 kHz

  /*
    configure(int configuration, char lidarliteAddress)

    Selects one of several preset configurations.

    Parameters
    ----------------------------------------------------------------------------
    configuration:  Default 0.
      0: Default mode, balanced performance.
      1: Short range, high speed. Uses 0x1d maximum acquisition count.
      2: Default range, higher speed short range. Turns on quick termination
          detection for faster measurements at short range (with decreased
          accuracy)
      3: Maximum range. Uses 0xff maximum acquisition count.
      4: High sensitivity detection. Overrides default valid measurement detection
          algorithm, and uses a threshold value for high sensitivity and noise.
      5: Low sensitivity detection. Overrides default valid measurement detection
          algorithm, and uses a threshold value for low sensitivity and noise.
    lidarliteAddress: Default 0x62. Fill in new address here if changed. See
      operating manual for instructions.
  */
  myLidarLite.configure(0); // Change this number to try out alternate configurations

  Initialize();

  FLUSHMOTORBUFFER();

}

///////////////////////////////////////////////////////////////////////////////////
////////////////////// Begin line finding ///////////////////////////////////////////

// Split the linear regresssion into 2 lines, and take the angle of the line with the highest RSquared
// Then rescan to find the line
void BinarySearchForBestLine(REAL x[], REAL y[], int n, double acceptableRSquared)
{
  REAL x1[MAX_READS];
  REAL x2[MAX_READS];
  REAL y1[MAX_READS];
  REAL y2[MAX_READS];

  int n1 = n/2;
  for(int i = 0; i<n/2; i++)
  {
    x1[i] = x[i];
    y1[i] = y[i];
  }

  REAL m1,b1,r1, rSquared1;
  rSquared1 = 0.0;
  int lineStatus = linreg(n1,x1,y1,&m1,&b1,&r1);
  rSquared1 = r1*r1;

  int n2 = n - n/2;
  for(int i = n/2; i<n; i++)
  {
    x2[i-n/2] = x[i];
    y2[i-n/2] = y[i];
  }

  REAL m2,b2,r2, rSquared2;
  rSquared2 = 0.0;
  lineStatus = linreg(n2,x2,y2,&m2,&b2,&r2);
  rSquared2 = r2*r2;

  if(rSquared1 > rSquared2)
  {
    sweepFrom = sweepFrom;
    sweepTo = (sweepTo-sweepFrom)/2 + sweepFrom;
  }
  else
  {
    sweepFrom = (sweepTo-sweepFrom)/2 + sweepFrom;
    sweepTo = sweepTo;
  }

  Serial.print("SweepTo: ");
  Serial.print(sweepTo);
  Serial.print(" SweepFrom: ");
  Serial.println(sweepFrom);
}

double Angle = 0.0;
double rSquared = 0.0;
double acceptableRSquared = 0.90;

double FindBestFitLineInDataSet(REAL x[], REAL y[], int n, int WallShouldBeOnRight)
{
  SweepError = false;
  int acceptableNumberOfReads = 3;
  double outlierThreshold = 30.0;
  int maxCycles = 5;
  REAL newX[MAX_READS];
  REAL newY[MAX_READS];

  REAL m,b,r;
  rSquared = 0.0;
  Angle = 0.0;
  int lineStatus = linreg(n,x,y,&m,&b,&r);

  if(lineStatus == 1)
  {
    SweepError = true;
  }
  else
  {
    Angle = atan(m)*180/3.14;
    rSquared = r*r;

    Serial.print("Angle: ");
    Serial.print(Angle);
    Serial.print(" rSquared: ");
    Serial.println(rSquared);

    if(rSquared > acceptableRSquared)
    {
      Serial.println("LINE DETECTED");
      SweepDone = true;
    }
    else
    {
      BinarySearchForBestLine(x, y, n, acceptableRSquared);
    }
  }

  return rSquared;
}

void stepForward()
{
  digitalWrite(dirPin, HIGH);
  digitalWrite(stepPin, HIGH);

  delay(stepTime/2);
  digitalWrite(stepPin, LOW);
  delay(stepTime/2);

  curStep++;
}

void stepBackward()
{
  digitalWrite(dirPin, LOW);
  digitalWrite(stepPin, HIGH);

  delay(stepTime/2);
  digitalWrite(stepPin, LOW);
  delay(stepTime/2);

  curStep--;
}

void Initialize()
{
  FoundZero = false;
  while(!FoundZero)
  {
    stepBackward();
  }

  curStep = 0;
}

double GetRadiansFromDegrees(double Degrees)
{
  return (Degrees/180)*3.14159;
}

void PrintSweepInfo()
{
  for(int i = 0; i<curDataPoints; i++)
  {
    double radians = GetRadiansFromDegrees(dataPoints[i].angle);
    double radius = dataPoints[i].reading;

    double x = radius*cos(radians);
    double y = radius*sin(radians);

    Serial.print("X: ");
    Serial.print(x);
    Serial.print(" Y: ");
    Serial.print(y);
    Serial.print(" Angle: ");
    Serial.print(dataPoints[i].angle);
    Serial.print(" Distance: ");
    Serial.println(dataPoints[i].reading);
  }
}

void PrintSweepXY()
{
  for(int i = 0; i<curDataPoints; i++)
  {
    double radians = GetRadiansFromDegrees(dataPoints[i].angle);
    double radius = dataPoints[i].reading;

    double x = radius*cos(radians);
    double y = radius*sin(radians);

    Serial.print(x);
    Serial.print((char)0x09);
    Serial.println(y);
  }
}

void FindLinesFromSweep(boolean OnRight)
{
  REAL xList[staticDataSet];
  REAL yList[staticDataSet];
  for(int i = 0; i < curDataPoints; i++)
  {
    double radians = GetRadiansFromDegrees(dataPoints[i].angle);
    double radius = dataPoints[i].reading;

    double x = radius*cos(radians);
    double y = radius*sin(radians);

    xList[i] = x;
    yList[i] = y;
  }

  FindBestFitLineInDataSet(xList, yList, curDataPoints, OnRight);
}

void Sweep(double fromAngle, double toAngle)
{
  Initialize();
  curDataPoints = 0;

  boolean done = false;

  while(!done)
  {
    stepForward();
    if(GetAngleFromStep(curStep) > fromAngle && GetAngleFromStep(curStep) < toAngle)
    {
      delay(10);
      Serial.print("Read at Angle: ");
      Serial.println(GetAngleFromStep(curStep));
      int distance = myLidarLite.distance();
      double Angle = GetAngleFromStep(curStep);

      LidarRead curRead;
      curRead.angle = Angle;
      curRead.reading = distance;
      dataPoints[curDataPoints] = curRead;
      curDataPoints++; 
    }
    else if(GetAngleFromStep(curStep) > toAngle)
    {
      Serial.println("End Sweep");
      done = true;
    }
  }
}

////////////////////// END LINE FINDING ////////////////////////////
///////////////////////////////////////////////////////////////////


int curWayPoint = 0;
boolean DataSent = false;
boolean sweeping = false;
boolean waitingForButton = false;
boolean buttonPressed = false;
boolean DoingAlgorithm = false;
boolean AlgorithmComplete = false;

double CorrectRightAngle = 8.0;
double CorrectLeftAngle = 9.0;
void DoCorrectionAngle(int findLineFrom, int findLineTo, boolean wallOnRight)
{
    if(!DataSent)
    {
      sweepFrom = findLineFrom;
      sweepTo = findLineTo;
      DataSent = true;
      sweeping = true;
      SweepDone = false;
    }

    Sweep(sweepFrom, sweepTo);
    PrintSweepXY();
    FindLinesFromSweep(wallOnRight);

    if(SweepDone)
    {
      DataSent = false;
      sweeping = false;
      if(!SweepError)
      {
        // Calculate new heading from detected angle
        if(rSquared > acceptableRSquared)
        {

        }
      }
    }
}

void OnCompleteWayPoint()
{
  curWayPoint++;
  DataSent = false;
  sweeping = false;
  waitingForButton = false;
  buttonPressed = false;
  DoingAlgorithm = false;
  AlgorithmComplete = false;

  Serial.print("Moving to Waypoint ");
  Serial.println(curWayPoint);
}

boolean ManualMode = false;
void DoSerialCommands()
{
  if(Serial.available())
  {
    Serial.println("Received Command");
    char cmdByte = Serial.read();
    if(cmdByte == '0')
    {
      ManualMode = true;
      int angleToMove = Serial.parseInt();
      Serial.print("Move to angle ");
      Serial.println(angleToMove);
      MoveMotorToAngle(angleToMove);
    }
    else if(cmdByte == '1')
    {
      ManualMode = true;
      float distToMove = Serial.parseFloat();
      Serial.print("Move to Pos ");
      Serial.println(distToMove);
      MoveMotorForward(distToMove);
    }
    else if(cmdByte == '2')
    {
      ManualMode = true;
      Serial.println("STOPPING BOT");
      StopMotor();
    }
    else if(cmdByte == '3')
    {
      ManualMode = false;
      Serial.println("Leaving Manual Mode");
    }
    else if(cmdByte == '4')
    {
      ManualMode = true;
      sweepFrom = Serial.parseFloat();
      Serial.print("SweepFrom: ");
      Serial.println(sweepFrom);
    }
    else if(cmdByte == '5')
    {
      ManualMode = true;
      sweepTo = Serial.parseFloat();
      Serial.print("SweepTo: ");
      Serial.println(sweepTo);
    }
    else if(cmdByte == '6')
    {
      ManualMode = true;
      DoCorrectionAngle(sweepFrom, sweepTo, true);
    }
    else if(cmdByte == '7')
    {
      ManualMode = true;
      DoCorrectionAngle(sweepFrom, sweepTo, false);
    }
    else if(cmdByte == '8') // Scan Left
    {
      sweepFrom = 140;
      sweepTo = 200;
      ManualMode = true;
      DoCorrectionAngle(sweepFrom, sweepTo, false);
    }
    else if(cmdByte == '9')
    {
      sweepFrom = 0;
      sweepTo = 80;
      ManualMode = true;
      DoCorrectionAngle(sweepFrom, sweepTo, true);
    }
  }
}

void IsButtonPressed()
{
  FLUSHMOTORBUFFER();
  waitingForButton = true;
  buttonPressed = false;
  digitalWrite(LED_PIN, HIGH);

  if(digitalRead(BTN_PIN) == 0)
  {
    buttonPressed = true;

    for(int i = 0; i < 5; i++)
    {
      digitalWrite(LED_PIN, LOW);
      delay(200); 
      digitalWrite(LED_PIN, HIGH);
      delay(200);
    }

    FLUSHMOTORBUFFER();
    digitalWrite(LED_PIN, LOW);
  }
}

/////////////////////////////////////////////////////////////////
///////////////// CONSTANT RUN TIME OBSTACLE AVOIDANCE //////////
long maxStep = 220;
boolean GoingUp = 0;
long minStep = 20;

void StepAndRead()
{
  if(curStep >= maxStep)
  {
    GoingUp = false;
  }
  else if(curStep <= minStep)
  {
    GoingUp = true;
  }

  if(GoingUp)
  {
    stepForward();
  }
  else
  {
    stepBackward();
  }

  int distance = myLidarLite.distance();
  double Angle = GetAngleFromStep(curStep);

  LidarRead curRead;
  curRead.angle = Angle;
  curRead.reading = distance;

  // Ignore if Distance == 1

  if(curRead.reading >= 2)
  {
    Serial.print("Angle: ");
    Serial.print(curRead.angle);
    Serial.print(" Reading: ");
    Serial.println(curRead.reading);

    int numValidReadings = 0;

    while(distance < 40 && distance >= 2)
    {
      numValidReadings++;

      distance = myLidarLite.distance();

      if(numValidReadings > 3) // 4 Valid readings needed to stop
      {
        StopMotor();
        ManualMode = true;

        while(distance < 40 || distance <= 2)
        {
          distance = myLidarLite.distance();
        }

        return;
      }
    }


  }

}


////////////////// END RUN TIME OBSTACLE AVOIDANCE //////////////
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////
////////////////////// ALGORITHMS //////////////////////////////////

double acceptableWallAngleDiff = 1.0;
double expectedWallOnRightAngle = -63;
void AlignToWallOnRight()
{
      DoingAlgorithm = true;
      AlgorithmComplete = false;
      
    do
    {
      sweepFrom = 0;
      sweepTo = 80;

      do
      {
        DoCorrectionAngle(sweepFrom, sweepTo, true);
      }while(!SweepDone && !SweepError);

      if(SweepError)
      {
        // Need Help
      }

      double angleDiff = Angle - expectedWallOnRightAngle;

      Serial.println(angleDiff);

      if(expectedWallOnRightAngle - acceptableWallAngleDiff < Angle && expectedWallOnRightAngle + acceptableWallAngleDiff > Angle)
      {
        DoingAlgorithm = false;
        AlgorithmComplete = true;
        break;
      }
      else
      {
        boolean dataSent = false;
        int prevSysAngle = curSystemAngle;
        FLUSHMOTORBUFFER();
        do
        {
          if(!dataSent)
            {
              dataSent = true;
              MoveMotorToAngle(prevSysAngle - angleDiff);
            }
        }while(!CheckForMotionComplete());
      }

    }while(DoingAlgorithm);
}

double expectedWallOnLeftAngle = -63;
void AlignToWallOnLeft()
{
      DoingAlgorithm = true;
      AlgorithmComplete = false;

      do
      {
        sweepFrom = 140;
        sweepTo = 200;

        do
        {
          DoCorrectionAngle(sweepFrom, sweepTo, true);
          Serial.print(SweepDone);
          Serial.print(" ");
          Serial.print(SweepError);
          Serial.print(" ");
          Serial.println(sweeping);
        }while(!SweepDone && !SweepError && sweeping);

        if(SweepError)
        {
          // Need Help
        }

        double angleDiff = Angle - expectedWallOnLeftAngle;

        Serial.println(angleDiff);

        if(expectedWallOnLeftAngle - acceptableWallAngleDiff < Angle && expectedWallOnLeftAngle + acceptableWallAngleDiff > Angle)
        {
          DoingAlgorithm = false;
          AlgorithmComplete = true;
          Serial.println("Found Acceptable Wall angle!");
          break;
        }
        else
        {
          int prevSysAngle = curSystemAngle;
          boolean dataSent = false;
          FLUSHMOTORBUFFER();
          do
          {
            if(!dataSent)
            {
              dataSent = true;
              MoveMotorToAngle(prevSysAngle - angleDiff);
            }
            
          }while(!CheckForMotionComplete());
        }

    }while(DoingAlgorithm);
}

void StepToSpecificPosition(int stepperAngle)
{
  while(curStep < stepperAngle)
  {
    stepForward();
  }
}

void DriveToCornerRight(int distance)
{
  Initialize();
  DoingAlgorithm = true;
  AlgorithmComplete = false;
  int stepperAngleToRight = 20;
  int allowableSlope = 10;
  int lastDistance = 0;
  boolean wallFound = false;

  Serial.println("Drive to Corner Right");

  StepToSpecificPosition(stepperAngleToRight);
  lastDistance = myLidarLite.distance();

  FLUSHMOTORBUFFER();
  MoveMotorForward(distance);

  do
  {
    int curDistance = myLidarLite.distance();
    Serial.print("Distance: ");
    Serial.println(curDistance);

    while(curDistance <= 2)
    {
      curDistance = myLidarLite.distance();
    }

    if(abs(curDistance - lastDistance) > allowableSlope)
    {
      StopMotor();
      wallFound = true;
    }

    lastDistance = curDistance;

    delay(50);
  }while(!wallFound);

  FLUSHMOTORBUFFER();

  AlgorithmComplete = true;
}

void DriveToCornerLeft(int distance)
{
  Initialize();
  DoingAlgorithm = true;
  AlgorithmComplete = false;
  int stepperAngleToLeft = 220;
  int allowableSlope = 10;
  int lastDistance = 0;
  boolean wallFound = false;

  Serial.println("Drive to Corner Left");

  StepToSpecificPosition(stepperAngleToLeft);
  lastDistance = myLidarLite.distance();

  FLUSHMOTORBUFFER();
  MoveMotorForward(distance);

  do
  {
    int curDistance = myLidarLite.distance();
    Serial.print("Distance: ");
    Serial.println(curDistance);

    while(curDistance <= 2)
    {
      curDistance = myLidarLite.distance();
    }

    if(abs(curDistance - lastDistance) > allowableSlope)
    {
      StopMotor();
      wallFound = true;
    }

    lastDistance = curDistance;

    delay(50);
  }while(!wallFound);

  FLUSHMOTORBUFFER();

  AlgorithmComplete = true;
}

void GetToWallDistanceRight(int wantedDistance)
{
  Initialize();
  DoingAlgorithm = true;
  AlgorithmComplete = false;
  int stepperAngleToRight = 20;
  int distanceMarginOfError = 10;
  boolean wallFound = false;

  Serial.println("Go To Wall Distance Right");

  // Read Wall
  StepToSpecificPosition(stepperAngleToRight);
  int dist = myLidarLite.distance();

  // Calculate difference in distance
  int distDiff = wantedDistance - dist;

  if(abs(distDiff) < distanceMarginOfError)
  {
    AlgorithmComplete = true;
    return;
  }

  Serial.print("ReadDist: ");
  Serial.print(dist);
  Serial.print(" DistDiff");
  Serial.println(distDiff);

  // Move to position in Right Angles 
  int MoveAngle = 0;
  if(distDiff < 0)
  {
    MoveAngle = curSystemAngle + 90;
    if(MoveAngle > 360)
      MoveAngle = MoveAngle - 360;
  }
  else
  {
    MoveAngle = curSystemAngle - 90;
    if(MoveAngle < 0)
      MoveAngle = MoveAngle + 360;
  }
  
  MoveMotorToAngle(MoveAngle);
  FLUSHMOTORBUFFER();
  while(!CheckForMotionComplete()) 
  {
    StepAndRead();
  }

  double distDiffF = abs(distDiff);
  distDiffF = distDiffF / 100;

  Serial.println(distDiffF);

  MoveMotorForward(distDiffF);
  FLUSHMOTORBUFFER();
  while(!CheckForMotionComplete()) 
  {
    StepAndRead();
  }

  if(distDiff < 0)
  {
    MoveAngle = curSystemAngle - 90;
    if(MoveAngle < 0)
      MoveAngle = MoveAngle + 360;
  }
  else
  {
    MoveAngle = curSystemAngle + 90;
    if(MoveAngle > 360)
      MoveAngle = MoveAngle - 360;
  }
  

  MoveMotorToAngle(MoveAngle);
  FLUSHMOTORBUFFER();
  while(!CheckForMotionComplete()) 
  {
    StepAndRead();
  }

  AlgorithmComplete = true;
  
}

void GetToWallDistanceLeft(int wantedDistance)
{
  Initialize();
  DoingAlgorithm = true;
  AlgorithmComplete = false;
  int stepperAngleToLeft = 220;
  int distanceMarginOfError = 10;
  boolean wallFound = false;

  Serial.println("Go To Wall Distance Left");

  // Read Wall
  StepToSpecificPosition(stepperAngleToLeft);
  int dist = myLidarLite.distance();

  // Calculate difference in distance
  int distDiff = wantedDistance - dist;

  if(abs(distDiff) < distanceMarginOfError)
  {
    AlgorithmComplete = true;
    return;
  }

  Serial.print("ReadDist: ");
  Serial.print(dist);
  Serial.print(" DistDiff");
  Serial.println(distDiff);

  // Move to position in Right Angles 
  int MoveAngle = 0;
  if(distDiff > 0)
  {
    MoveAngle = curSystemAngle + 90;
    if(MoveAngle > 360)
      MoveAngle = MoveAngle - 360;
  }
  else
  {
    MoveAngle = curSystemAngle - 90;
    if(MoveAngle < 0)
      MoveAngle = MoveAngle + 360;
  }
  
  MoveMotorToAngle(MoveAngle);
  FLUSHMOTORBUFFER();
  while(!CheckForMotionComplete()) 
  {
    StepAndRead();
  }

  double distDiffF = abs(distDiff);
  distDiffF = distDiffF / 100;

  Serial.println(distDiffF);

  MoveMotorForward(distDiffF);
  FLUSHMOTORBUFFER();
  while(!CheckForMotionComplete()) 
  {
    StepAndRead();
  } 

  if(distDiff > 0)
  {
    MoveAngle = curSystemAngle - 90;
    if(MoveAngle < 0)
      MoveAngle = MoveAngle + 360;
  }
  else
  {
    MoveAngle = curSystemAngle + 90;
    if(MoveAngle > 360)
      MoveAngle = MoveAngle - 360;
  }
  

  MoveMotorToAngle(MoveAngle);
  FLUSHMOTORBUFFER();
  while(!CheckForMotionComplete()) 
  {
    StepAndRead();
  }

  AlgorithmComplete = true;
  
}

/////////////////////// END ALGORITHMS //////////////////////////////
////////////////////////////////////////////////////////////////////
void DoSquares()
{
  if(!ManualMode)
  {
    if(curWayPoint == 1)
    {
      if(!DataSent && !ManualMode)
      {
        FLUSHMOTORBUFFER();
        MoveMotorToAngle(78);
        DataSent = true;
        sweeping = false;
      }
    }
    else if(curWayPoint == 2)
    {
      if(!DataSent && !ManualMode)
      {
        FLUSHMOTORBUFFER();
        MoveMotorForward(3);
        DataSent = true;
        sweeping = false;
        FLUSHMOTORBUFFER();
      }
    }
    else if(curWayPoint == 3)
    {
      if(!DataSent && !ManualMode)
      {
        FLUSHMOTORBUFFER();
        MoveMotorToAngle(168);
        DataSent = true;
        sweeping = false;
      }
    }
    else if(curWayPoint == 4)
    {
      if(!DataSent && !ManualMode)
      {
        FLUSHMOTORBUFFER();
        MoveMotorForward(3);
        DataSent=true;
        sweeping = false;
      }
    }
    else if(curWayPoint == 5)
    {
      //DoCorrectionAngle(0, 80, true);
      if(!DataSent && !ManualMode)
      {
        FLUSHMOTORBUFFER();
        MoveMotorToAngle(258);
        DataSent = true;
        sweeping = false;
      }
    }
    else if(curWayPoint == 6)
    {
      if(!DataSent && !ManualMode)
      {
        FLUSHMOTORBUFFER();
        MoveMotorForward(3);
        DataSent = true;
        sweeping = false;
      }
    }
    else if(curWayPoint == 7)
    {
      if(!DataSent && !ManualMode)
      {
        FLUSHMOTORBUFFER();
        MoveMotorToAngle(348);
        DataSent = true;
        sweeping = false;
      }
    }
    else if(curWayPoint == 8)
    {
      if(!DataSent && !ManualMode)
      {
        FLUSHMOTORBUFFER();
        MoveMotorForward(3);
        DataSent = true;
        sweeping = false;
      }
    }

    if(curWayPoint >= 8)
    {
      curWayPoint = 1;
    }
  }
  

}

void DoWallFindingSquares()
{
  if(!ManualMode)
  {
    if(curWayPoint == 1)
    {
      if(!DataSent && !ManualMode)
      {
        FLUSHMOTORBUFFER();
        MoveMotorToAngle(78);
        DataSent = true;
        sweeping = false;
      }
    }
    if(curWayPoint == 2)
    {
      AlignToWallOnLeft();
    }
    if(curWayPoint == 3)
    {
      if(!DataSent && !ManualMode)
      {
        FLUSHMOTORBUFFER();
        MoveMotorForward(4);
        DataSent = true;
        sweeping = false;
      }
    }
    if(curWayPoint == 4)
    {
      if(!DataSent && !ManualMode)
      {
        FLUSHMOTORBUFFER();
        MoveMotorToAngle(curSystemAngle+90);
        DataSent = true;
        sweeping = false;
      }
    }
    if(curWayPoint == 5)
    {
      if(!DataSent && !ManualMode)
      {
        FLUSHMOTORBUFFER();
        MoveMotorForward(4);
        DataSent = true;
        sweeping = false;
      }
    }
  }
}

void DriveToCornerTest()
{
  if(curWayPoint == 1)
  {
    if(!DataSent && !ManualMode)
    {
      FLUSHMOTORBUFFER();
      MoveMotorToAngle(78);
      DataSent = true;
      sweeping = false;
    }
  }
  if(curWayPoint == 2)
  {
    AlignToWallOnLeft();
  }
  if(curWayPoint == 3)
  {
    DriveToCornerLeft(5);
  }
}

void TurnToAngleComplete(int angle)
{
  if(!DataSent && !ManualMode)
  {
    if(angle > 360)
      angle = angle - 360;
    else if(angle < 0)
      angle = angle + 360;

    FLUSHMOTORBUFFER();
    MoveMotorToAngle(angle);
    FLUSHMOTORBUFFER();
    DataSent = true;
    sweeping = false;
  }
}

void GoToPositionComplete(float meters)
{
    if(!DataSent && !ManualMode)
    {
      FLUSHMOTORBUFFER();
      MoveMotorForward(meters);
      FLUSHMOTORBUFFER();
      DataSent = true;
      sweeping = false;
    }
}

int maxWayPointPizza = 0;
void PioneerWorksFromTrashCans()
{
  if(curWayPoint == maxWayPointPizza + 1)
  {
    TurnToAngleComplete(327);
  }
  else if(curWayPoint == maxWayPointPizza + 2)
  {
    AlignToWallOnLeft();
  }
  else if(curWayPoint == maxWayPointPizza + 3)
  {
    GetToWallDistanceLeft(250);
  }
  else if(curWayPoint == maxWayPointPizza + 4)
  {
    GoToPositionComplete(5); // 5m past trash cans
  }
  else if(curWayPoint == maxWayPointPizza + 5)
  {
    GetToWallDistanceLeft(250);
  }
  else if(curWayPoint == maxWayPointPizza + 6)
  {
    GoToPositionComplete(5); // 10m past trash cans
  }
  else if(curWayPoint == maxWayPointPizza + 7)
  {
    GetToWallDistanceLeft(250); 
  }
  else if(curWayPoint == maxWayPointPizza + 8)
  {
    GoToPositionComplete(5); //15m past trash cans
  }
  else if(curWayPoint == maxWayPointPizza + 9)
  {
    GetToWallDistanceLeft(200);
  }
  else if(curWayPoint == maxWayPointPizza + 10)
  {
    GoToPositionComplete(5); // 20m past trash cans
  }
  else if(curWayPoint == maxWayPointPizza + 11)
  {
    GetToWallDistanceLeft(200);
  }
  else if(curWayPoint == maxWayPointPizza + 12)
  {
    GoToPositionComplete(5); // 25 m past trash cans
  }
  else if(curWayPoint == maxWayPointPizza + 13)
  {
    GetToWallDistanceLeft(200);
  }
  else if(curWayPoint == maxWayPointPizza + 14)
  {
    GoToPositionComplete(5); // 30 m past trash cans
  }
  else if(curWayPoint == maxWayPointPizza + 15)
  {
    GetToWallDistanceLeft(200);
  }
  else if(curWayPoint == maxWayPointPizza + 16)
  {
    GoToPositionComplete(5); // 35 m past trash cans
  }
  else if(curWayPoint == maxWayPointPizza + 17)
  {
    GetToWallDistanceLeft(200);
  }
  else if(curWayPoint == maxWayPointPizza + 18)
  {
    GoToPositionComplete(5); // 40 m past trash cans
  }
  else if(curWayPoint == maxWayPointPizza + 19)
  {
    GetToWallDistanceLeft(200);
  }
  else if(curWayPoint == maxWayPointPizza + 20)
  {
    GoToPositionComplete(5); // 45 m past trash cans
  }
  else if(curWayPoint == maxWayPointPizza + 21)
  {
    GetToWallDistanceLeft(200);
  }
  else if(curWayPoint == maxWayPointPizza + 22)
  {
    GoToPositionComplete(5); // 50 m past trash cans
  }
  else if(curWayPoint == maxWayPointPizza + 23)
  {
    GetToWallDistanceLeft(200);
  }
  else if(curWayPoint == maxWayPointPizza + 24)
  {
    GoToPositionComplete(5); // 55 m past trash cans
  }
  else if(curWayPoint == maxWayPointPizza + 25)
  {
    GetToWallDistanceLeft(200);
  }
  else if(curWayPoint == maxWayPointPizza + 26)
  {
    GoToPositionComplete(5); // 60 m past trash cans
  }
  else if(curWayPoint == maxWayPointPizza + 27)
  {
    GetToWallDistanceLeft(200);
  }
  else if(curWayPoint == maxWayPointPizza + 28)
  {
    GoToPositionComplete(5); // 65 m past trash cans
  }
  else if(curWayPoint == maxWayPointPizza + 29)
  {
    TurnToAngleComplete(curSystemAngle - 90); // Turn toward pioneer works
  }
  else if(curWayPoint == maxWayPointPizza + 30)
  {
    IsButtonPressed();
  }
  else if(curWayPoint == maxWayPointPizza + 31)
  {
    GoToPositionComplete(4); // Drive into pioneer works
  }
}

void PioneerWorksFromPizzaShop()
{
  if(curWayPoint == maxWayPointPizza + 1)
  {
    IsButtonPressed();
  }
  else if(curWayPoint == maxWayPointPizza + 2)
  {
    TurnToAngleComplete(curSystemAngle + 90);
  }
  else if(curWayPoint == maxWayPointPizza + 3)
  {
    GoToPositionComplete(5); // 5m
  }
  else if(curWayPoint == maxWayPointPizza + 4)
  {
    GetToWallDistanceLeft(200);
  }
  else if(curWayPoint == maxWayPointPizza + 5)
  {
    GoToPositionComplete(5); // 10m
  }
  else if(curWayPoint == maxWayPointPizza + 6)
  {
    GetToWallDistanceLeft(200);
  }
  else if(curWayPoint == maxWayPointPizza + 7)
  {
    GoToPositionComplete(4); // 14m
  }
  else if(curWayPoint == maxWayPointPizza + 8)
  {
    GetToWallDistanceLeft(200);
  }
  else if(curWayPoint == maxWayPointPizza + 9)
  {
    IsButtonPressed();
  }


  else if(curWayPoint == maxWayPointPizza + 10)
  {
    DriveToCornerLeft(5); // Find Corner
  }
  else if(curWayPoint == maxWayPointPizza + 11)
  {
    IsButtonPressed();
  }
  else if(curWayPoint == maxWayPointPizza + 12)
  {
    GoToPositionComplete(2);
  }
  else if(curWayPoint == maxWayPointPizza + 13)
  {
    TurnToAngleComplete(curSystemAngle - 90);
  }
  else if(curWayPoint == maxWayPointPizza + 14)
  {
    GoToPositionComplete(2); // 2m into journey
  }
  else if(curWayPoint == maxWayPointPizza + 15)
  {
    GetToWallDistanceLeft(200);
  }
  else if(curWayPoint == maxWayPointPizza + 16) 
  {
    GoToPositionComplete(5); // 7m into Journey
  }
  else if(curWayPoint == maxWayPointPizza + 17)
  {
    GetToWallDistanceLeft(250);
  }
  else if(curWayPoint == maxWayPointPizza + 18)
  {
    DriveToCornerLeft(5); // Find Trash Cans
  }
  else if(curWayPoint == maxWayPointPizza + 19)
  {
    GoToPositionComplete(1); // 1m into trashcan
  }
  else if(curWayPoint == maxWayPointPizza + 20)
  {
    GetToWallDistanceLeft(125);
  }
  else if(curWayPoint == maxWayPointPizza + 21)
  {
    GoToPositionComplete(3); // 4m into trashcan
  }
  else if(curWayPoint == maxWayPointPizza + 22)
  {
    GetToWallDistanceLeft(125);
  }
  else if(curWayPoint == maxWayPointPizza + 23)
  {
    DriveToCornerLeft(5); // Find end of trashcans.... 67.55 to go
  }
  else if(curWayPoint == maxWayPointPizza + 24)
  {
    GoToPositionComplete(5); // 5m past trash cans
  }
  else if(curWayPoint == maxWayPointPizza + 25)
  {
    GetToWallDistanceLeft(250);
  }
  else if(curWayPoint == maxWayPointPizza + 26)
  {
    GoToPositionComplete(5); // 10m past trash cans
  }
  else if(curWayPoint == maxWayPointPizza + 27)
  {
    GetToWallDistanceLeft(250); 
  }
  else if(curWayPoint == maxWayPointPizza + 28)
  {
    GoToPositionComplete(5); //15m past trash cans
  }
  else if(curWayPoint == maxWayPointPizza + 29)
  {
    GetToWallDistanceLeft(200);
  }
  else if(curWayPoint == maxWayPointPizza + 30)
  {
    GoToPositionComplete(5); // 20m past trash cans
  }
  else if(curWayPoint == maxWayPointPizza + 31)
  {
    GetToWallDistanceLeft(200);
  }
  else if(curWayPoint == maxWayPointPizza + 32)
  {
    GoToPositionComplete(5); // 25 m past trash cans
  }
  else if(curWayPoint == maxWayPointPizza + 33)
  {
    GetToWallDistanceLeft(200);
  }
  else if(curWayPoint == maxWayPointPizza + 34)
  {
    GoToPositionComplete(5); // 30 m past trash cans
  }
  else if(curWayPoint == maxWayPointPizza + 35)
  {
    GetToWallDistanceLeft(200);
  }
  else if(curWayPoint == maxWayPointPizza + 36)
  {
    GoToPositionComplete(5); // 35 m past trash cans
  }
  else if(curWayPoint == maxWayPointPizza + 37)
  {
    GetToWallDistanceLeft(200);
  }
  else if(curWayPoint == maxWayPointPizza + 38)
  {
    GoToPositionComplete(5); // 40 m past trash cans
  }
  else if(curWayPoint == maxWayPointPizza + 39)
  {
    GetToWallDistanceLeft(200);
  }
  else if(curWayPoint == maxWayPointPizza + 40)
  {
    GoToPositionComplete(5); // 45 m past trash cans
  }
  else if(curWayPoint == maxWayPointPizza + 41)
  {
    GetToWallDistanceLeft(200);
  }
  else if(curWayPoint == maxWayPointPizza + 42)
  {
    GoToPositionComplete(5); // 50 m past trash cans
  }
  else if(curWayPoint == maxWayPointPizza + 43)
  {
    GetToWallDistanceLeft(200);
  }
  else if(curWayPoint == maxWayPointPizza + 44)
  {
    GoToPositionComplete(5); // 55 m past trash cans
  }
  else if(curWayPoint == maxWayPointPizza + 45)
  {
    GetToWallDistanceLeft(200);
  }
  else if(curWayPoint == maxWayPointPizza + 46)
  {
    GoToPositionComplete(5); // 60 m past trash cans
  }
  else if(curWayPoint == maxWayPointPizza + 47)
  {
    GetToWallDistanceLeft(200);
  }
  else if(curWayPoint == maxWayPointPizza + 48)
  {
    GoToPositionComplete(5); // 65 m past trash cans
  }
  else if(curWayPoint == maxWayPointPizza + 49)
  {
    TurnToAngleComplete(curSystemAngle - 90); // Turn toward pioneer works
  }
  else if(curWayPoint == maxWayPointPizza + 50)
  {
    IsButtonPressed();
  }
  else if(curWayPoint == maxWayPointPizza + 51)
  {
    GoToPositionComplete(4); // Drive into pioneer works
  }
}

void PizzaFromTrashCans()
{
  maxWayPointPizza = 19;

  if(curWayPoint == 1)
  {
    TurnToAngleComplete(60);
  }
  else if(curWayPoint == 2)
  {
    AlignToWallOnRight();
  }
  else if(curWayPoint == 3)
  {
    GetToWallDistanceRight(200);
  }
  else if(curWayPoint == 4)
  {
    IsButtonPressed();
  }
  else if(curWayPoint == 5)
  {
    DriveToCornerRight(10); // Find corner of Pioneer and Van Brunt
  }
  else if(curWayPoint == 6)
  {
    IsButtonPressed();
  }
  else if(curWayPoint == 7)
  {
    GoToPositionComplete(1.7); // Move into sideway
  }
  else if(curWayPoint == 8)
  {
    IsButtonPressed();
  }
  else if(curWayPoint == 9)
  {
    TurnToAngleComplete(curSystemAngle+90); // Rotate Toward Marks
  }
  else if(curWayPoint == 10)
  {
    GoToPositionComplete(2); // Go 2m toward the deli
  }
  else if(curWayPoint == 11)
  {
    GetToWallDistanceRight(200);
  }
  else if(curWayPoint == 12)
  {
    GoToPositionComplete(3); // 5m from corner
  }
  else if(curWayPoint == 13)
  {
    GetToWallDistanceRight(200);
  }
  else if(curWayPoint == 14)
  {
    GoToPositionComplete(3); // 8m from corner
  }
  else if(curWayPoint == 15)
  {
    GetToWallDistanceRight(200);
  }
  else if(curWayPoint == 16)
  {
    GoToPositionComplete(5); //13m from corner
  }
  else if(curWayPoint == 17)
  {
    GetToWallDistanceRight(200);
  }
  else if(curWayPoint == 18)
  {
    GoToPositionComplete(3.6); // 16.6m from corner .....   AT MARKS
  }
  else if(curWayPoint == 19)
  {
    TurnToAngleComplete(curSystemAngle+90); // Look into marks pizza
  }

}

void GoToPizzaShop()
{
  maxWayPointPizza = 47;

  if(curWayPoint == 1) // Wait for Button Press
  {
    IsButtonPressed();
  }
  else if(curWayPoint == 2) // Turn toward Pioneer Works door
  {
    TurnToAngleComplete(60);
  }
  else if(curWayPoint == 3) // Leave through pioneer works garage - 4m
  {
    GoToPositionComplete(4);
  }
  else if(curWayPoint == 4) // Turn to look up Pioneer St
  {
    TurnToAngleComplete(curSystemAngle + 90);
  }
  else if(curWayPoint == 5) // Clear the garage door
  {
    GoToPositionComplete(3); // 3m into journey
  }
  else if(curWayPoint == 6) // Wait for button press after garage door lowered
  {
    IsButtonPressed();
  }
  else if(curWayPoint == 7) // Align to the wall on right
  {
    AlignToWallOnRight();
  }
  else if(curWayPoint == 8) // Get distance to wall
  {
    GetToWallDistanceRight(200);
  }
  else if(curWayPoint == 9) // start to move
  {
    GoToPositionComplete(10); // 13m into journey
  }
  else if(curWayPoint == 10)
  {
    GetToWallDistanceRight(200);
  }
  else if(curWayPoint == 11)
  {
    GoToPositionComplete(10); // 23m into journey
  }
  else if(curWayPoint == 12)
  {
    GetToWallDistanceRight(200);
  }
  else if(curWayPoint == 13)
  {
    GoToPositionComplete(10); // 33m
  }
  else if(curWayPoint == 14)
  {
    GetToWallDistanceRight(200);
  }
  else if(curWayPoint == 15)
  {
    GoToPositionComplete(10); // 43m
  }
  else if(curWayPoint == 16)
  {
    GetToWallDistanceRight(200);
  }
  else if(curWayPoint == 17)
  {
    GoToPositionComplete(10); // 53m
  }
  else if(curWayPoint == 18)
  {
    GetToWallDistanceRight(200);
  }
  else if(curWayPoint == 19) // MOVE DIFFERENTLY BECAUSE OF LUMP
  {
    GoToPositionComplete(5); // 58m
  }
  else if(curWayPoint == 20)
  {
    GetToWallDistanceRight(250);
    //TurnToAngleComplete(155);
  }
  else if(curWayPoint == 21)
  {
    GoToPositionComplete(6); // 64m
    //AlignToWallOnRight();
  }
  else if(curWayPoint == 22)
  {
    IsButtonPressed();
  }
  else if(curWayPoint == 23)
  {
    DriveToCornerRight(10); // This will end once hitting the wood stuff.. 15.5m from corner to Marks
  }
  else if(curWayPoint == 24)
  {
    IsButtonPressed();
  }
  else if(curWayPoint == 25)
  {
    GoToPositionComplete(1); // 1m into trash cans
  }
  else if(curWayPoint == 26)
  {
    GetToWallDistanceRight(125);
  }
  else if(curWayPoint == 27)
  {
    GoToPositionComplete(3);
  }
  else if(curWayPoint == 28)
  {
    GetToWallDistanceRight(125);
  }


  else if(curWayPoint == 29)
  {
    DriveToCornerRight(5); // Find end of trash cans
  }
  else if(curWayPoint == 30)
  {
    GoToPositionComplete(1);
  }
  else if(curWayPoint == 31)
  {
    GetToWallDistanceRight(200);
  }
  else if(curWayPoint == 32)
  {
    IsButtonPressed();
  }
  else if(curWayPoint == 33)
  {
    DriveToCornerRight(10); // Find corner of Pioneer and Van Brunt
  }
  else if(curWayPoint == 34)
  {
    IsButtonPressed();
  }
  else if(curWayPoint == 35)
  {
    GoToPositionComplete(1.7); // Move into sideway
  }
  else if(curWayPoint == 36)
  {
    IsButtonPressed();
  }
  else if(curWayPoint == 37)
  {
    TurnToAngleComplete(curSystemAngle+90); // Rotate Toward Marks
  }
  else if(curWayPoint == 38)
  {
    GoToPositionComplete(2); // Go 2m toward the deli
  }
  else if(curWayPoint == 39)
  {
    GetToWallDistanceRight(200);
  }
  else if(curWayPoint == 40)
  {
    GoToPositionComplete(3); // 5m from corner
  }
  else if(curWayPoint == 41)
  {
    GetToWallDistanceRight(200);
  }
  else if(curWayPoint == 42)
  {
    GoToPositionComplete(3); // 8m from corner
  }
  else if(curWayPoint == 43)
  {
    GetToWallDistanceRight(200);
  }
  else if(curWayPoint == 44)
  {
    GoToPositionComplete(5); //13m from corner
  }
  else if(curWayPoint == 45)
  {
    GetToWallDistanceRight(200);
  }
  else if(curWayPoint == 46)
  {
    GoToPositionComplete(3.6); // 16.6m from corner .....   AT MARKS
  }
  else if(curWayPoint == 47)
  {
    TurnToAngleComplete(curSystemAngle+90); // Look into marks pizza
  }
  
}

void TestWallMotion()
{
  if(curWayPoint == 1)
  {
    IsButtonPressed();
  }
  else if(curWayPoint == 2)
  {
    TurnToAngleComplete(238);
  }
  else if(curWayPoint == 3)
  {
    GetToWallDistanceRight(200);
  }
  else if(curWayPoint == 4)
  {
    IsButtonPressed();
  }
  else if(curWayPoint == 5)
  {
    TurnToAngleComplete(curSystemAngle - 180);
  }
  else if(curWayPoint == 6)
  {
    GetToWallDistanceLeft(500);
  }
  else if(curWayPoint == 7)
  {
    GetToWallDistanceLeft(300);
  }
  else if(curWayPoint == 8)
  {
    IsButtonPressed();
  }
  else if(curWayPoint == 9)
  {
    TurnToAngleComplete(curSystemAngle + 180);
  }
  else if(curWayPoint == 10)
  {
    GetToWallDistanceRight(600);
  }
}

void NavigatePioneer()
{
  if(curWayPoint == 1)
  {
    IsButtonPressed();
  }
  else if(curWayPoint == 2)
  {
    TurnToAngleComplete(60);
  }
  else if(curWayPoint == 3)
  {
    GetToWallDistanceRight(200);
  }
  else if(curWayPoint == 4)
  {
    AlignToWallOnRight();
  }
  else if(curWayPoint == 5)
  {
    GoToPositionComplete(5);
  }
  else if(curWayPoint == 6)
  {
    GetToWallDistanceRight(200);
  }
  else if(curWayPoint == 7)
  {
    GoToPositionComplete(5);
  }
}

boolean sent = true;

void loop()
{
  DoSerialCommands();

  if(sent == false)
  {
    curWayPoint = 20;
    sent = true;
  }

  // EXECUTE YOUR PROGRAMS INSIDE OF HERE
  if(!ManualMode)
  {
    //DoWallFindingSquares();
    //DriveToCornerTest();
    //DoSquares();
    //TestWallMotion();
    //NavigatePioneer();
    GoToPizzaShop();
    PioneerWorksFromPizzaShop();
  }

  
  
  // if(curWayPoint == 0)
  // {
  //   DoCorrectionAngle(160, 200, true);
  // }

  

  // if(curWayPoint == 1)
  // {
  //   if(!DataSent && !ManualMode)
  //   {
  //     FLUSHMOTORBUFFER();
  //     MoveMotorToAngle(78);
  //     DataSent = true;
  //     sweeping = false;
  //   }
  // }
  // if(curWayPoint == 2)
  // {
  //   AlignToWallOnLeft();
  // }
  // if(curWayPoint == 3)
  // {
  //   if(!DataSent && !ManualMode)
  //   {
  //     FLUSHMOTORBUFFER();
  //     MoveMotorForward(4);
  //     DataSent = true;
  //     sweeping = false;
  //   }
  // }
  // if(curWayPoint == 4)
  // {
  //   if(!DataSent && !ManualMode)
  //   {
  //     FLUSHMOTORBUFFER();
  //     MoveMotorToAngle(258);
  //     DataSent = true;
  //     sweeping = false;
  //   }
  // }
  // if(curWayPoint == 5)
  // {
  //   AlignToWallOnRight();
  // }
  // if(curWayPoint == 6)
  // {
  //   if(!DataSent && !ManualMode)
  //   {
  //     FLUSHMOTORBUFFER();
  //     MoveMotorForward(4);
  //     DataSent = true;
  //     sweeping = false;
  //   }
  // }



  // else if(curWayPoint == 3)
  // {
  //   if(!DataSent && !ManualMode)
  //   {
  //     FLUSHMOTORBUFFER();
  //     MoveMotorToAngle(258);
  //     DataSent = true;
  //     sweeping = false;
  //   }
  // }
  // else if(curWayPoint == 4)
  // {
  //   AlignToWallOnRight();
  // }

  // if(curWayPoint == 1)
  // {
  //   if(!DataSent && !ManualMode)
  //   {
  //     FLUSHMOTORBUFFER();
  //     MoveMotorToAngle(222);
  //     DataSent = true;
  //     sweeping = false;
  //   }
  // }
  // else if(curWayPoint == 2)
  // {
  //   if(!DataSent && !ManualMode)
  //   {
  //     FLUSHMOTORBUFFER();
  //     MoveMotorForward(3);
  //     DataSent = true;
  //     sweeping = false;
  //     FLUSHMOTORBUFFER();
  //   }
  // }
  // else if(curWayPoint == 3)
  // {
  //   if(!DataSent && !ManualMode)
  //   {
  //     FLUSHMOTORBUFFER();
  //     MoveMotorToAngle(312);
  //     MoveMotorForward(2);
  //     DataSent = true;
  //     sweeping = false;
  //   }
  // }
  // else if(curWayPoint == 4)
  // {
  //   if(!DataSent)
  //   {
  //     FLUSHMOTORBUFFER();
  //     MoveMotorForward(3);
  //     DataSent=true;
  //     sweeping = false;
  //   }
  // }
  // else if(curWayPoint == 5)
  // {
  //   //DoCorrectionAngle(0, 80, true);
  //   if(!DataSent)
  //   {
  //     FLUSHMOTORBUFFER();
  //     MoveMotorToAngle(42);
  //     DataSent = true;
  //     sweeping = false;
  //   }
  // }
  // else if(curWayPoint == 6)
  // {
  //   if(!DataSent)
  //   {
  //     FLUSHMOTORBUFFER();
  //     MoveMotorForward(3);
  //     DataSent = true;
  //     sweeping = false;
  //   }
  // }
  // else if(curWayPoint == 7)
  // {
  //   if(!DataSent)
  //   {
  //     FLUSHMOTORBUFFER();
  //     MoveMotorToAngle(132);
  //     DataSent = true;
  //     sweeping = false;
  //   }
  // }
  // else if(curWayPoint == 8)
  // {
  //   if(!DataSent)
  //   {
  //     FLUSHMOTORBUFFER();
  //     MoveMotorForward(3);
  //     DataSent = true;
  //     sweeping = false;
  //   }
  // }
  // else if(curWayPoint == 6)
  // {
  //   if(!DataSent)
  //   {
  //     FLUSHMOTORBUFFER();
  //     MoveMotorForward(2);
  //     DataSent = true;
  //     sweeping = false;
  //   }
  // }
  // else if(curWayPoint == 7)
  // {
  //   if(!DataSent)
  //   {
  //     FLUSHMOTORBUFFER();
  //     MoveMotorToAngle(350);
  //     DataSent = true;
  //     sweeping = false;
  //   }
  // }
  // else if(curWayPoint == 8)
  // {
  //   if(!DataSent)
  //   {
  //     FLUSHMOTORBUFFER();
  //     MoveMotorForward(2);
  //     DataSent = true;
  //     sweeping = false;
  //   }
  // }
  
  // if(!SweepDone && !SweepError)
  // {
  //   Sweep(sweepFrom, sweepTo);
  //   //PrintSweepInfo();
  //   FindLinesFromSweep(true);
  //   PrintSweepXY();
  // }
  // else if(SweepError)
  // {
  //   sweepTo = 90;
  //   sweepFrom = 0;
  //   Serial.println("SWEEP ERROR");
  // }

  if(!sweeping && !waitingForButton && curWayPoint > 0)
  {
    StepAndRead();
  }

  if(CheckForMotionComplete() && !sweeping && !waitingForButton && !DoingAlgorithm)
  {
    // Move on to next motion
    OnCompleteWayPoint();
  }
  else if(sweeping && SweepDone)
  {
    OnCompleteWayPoint();
  }
  else if(waitingForButton && buttonPressed)
  {
     OnCompleteWayPoint();
  }
  else if(DoingAlgorithm && AlgorithmComplete)
  {
    OnCompleteWayPoint();
  }
}


void fullRotation(){
  FoundZero = true;
}


