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

float wheelBase = 502;
float wheelCircumference = 812;     //based on measuring w a wire (was originally 796)
float numOfSteps = 185;
float stepDist = wheelCircumference/numOfSteps;

int rHitTarget = 0;
int lHitTarget = 0;

const byte interruptPin1 = 2;
const byte interruptPin2 = 3;
int rTarget = -10;
int lTarget = -10;



void setup() {

  Serial.begin(9600);
 
 
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
  delay(500);
  digitalWrite(led, LOW);
  delay(5000);
}


void loop() {



//setWheelSpeeds();

//goToTarget();
  
//printInfo();            //nice looking display w labels
//infoToProcessing();       //CSV for processing

//newTarget(50);
calcAngleCoordinates();
while(botAngle*57.3 < 360){
    digitalWrite(dirPinR, LOW);               //FWD
    analogWrite(spdPinR, 30);
    digitalWrite(dirPinL, HIGH);               //FWD
    analogWrite(spdPinL, 30);
    calcAngleCoordinates();
    Serial.println(botAngle*57.3);
}
    analogWrite(spdPinL, 0);
    analogWrite(spdPinR, 0);

}

void goToAngle(int a){
  
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


void goToTarget(){
  if (rWheelTicks <rTarget) {
    digitalWrite(dirPinR, LOW);               //FWD
    analogWrite(spdPinR, lWheelSpeed);
  }
  else if (rWheelTicks > rTarget) {
    digitalWrite(dirPinR, HIGH);              //REV
    analogWrite(spdPinR, lWheelSpeed);
  }
  else {
    analogWrite(spdPinR, 0);
    rHitTarget = 1;
  }

  if (lWheelTicks < lTarget) {
    digitalWrite(dirPinL, LOW);
    analogWrite(spdPinL, lWheelSpeed);
  }
  else if (lWheelTicks > lTarget) {
    digitalWrite(dirPinL, HIGH);
    analogWrite(spdPinL, lWheelSpeed);
  }
  else {
    analogWrite(spdPinL, 0);
    lHitTarget = 1;
  }
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


void calcAngleCoordinates() {

  rWheelDist = rWheelChange * stepDist;
  lWheelDist = lWheelChange * stepDist;
  angleChange = (rWheelDist - lWheelDist) / wheelBase;
  botAngle = botAngle + angleChange;
  distCenter = (rWheelDist + lWheelDist)/2; 
  botX = botX + distCenter*cos(botAngle);
  botY = botY + distCenter*sin(botAngle);

  
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
