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

LIDARLite myLidarLite;

int dirPin = 2;
int stepPin = 3;
int stepTime = 2;

int optoPin = 11;

int num_Readings = 0;

#define stepsForFullRotation 5000

int dataPoints[stepsForFullRotation];

void setup()
{
  delay(1000);
  
  Serial.begin(115200); // Initialize serial connection to display distance readings

  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  
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

}

void stepForward()
{
  digitalWrite(dirPin, HIGH);
  digitalWrite(stepPin, HIGH);

  delay(stepTime/2);
  digitalWrite(stepPin, LOW);
  delay(stepTime/2);
}

void stepBackward()
{
  digitalWrite(dirPin, LOW);
  digitalWrite(stepPin, HIGH);

  delay(stepTime/2);
  digitalWrite(stepPin, LOW);
  delay(stepTime/2);
}

void loop()
{
  /*
    distance(bool biasCorrection, char lidarliteAddress)

    Take a distance measurement and read the result.

    Parameters
    ----------------------------------------------------------------------------
    biasCorrection: Default true. Take aquisition with receiver bias
      correction. If set to false measurements will be faster. Receiver bias
      correction must be performed periodically. (e.g. 1 out of every 100
      readings).
    lidarliteAddress: Default 0x62. Fill in new address here if changed. See
      operating manual for instructions.
  */
//    Serial.println("About to if");
    if(num_Readings % 100 == 0) dataPoints[num_Readings] = myLidarLite.distance();
    else dataPoints[num_Readings] = myLidarLite.distance(false);

//    Serial.println("About to Step Forward");
    stepForward();
//    Serial.println(num_Readings);
    num_Readings++;
  

}


void sendData(){
  
  for(int i = 0; i < num_Readings; i++){
    Serial.print(dataPoints[i]);
    if(i != num_Readings - 1 ) Serial.print(",");
  }
  Serial.println();

}

void fullRotation(){
sendData();
//    Serial.print(num_Readings);
//    Serial.println(": OPTO");
    num_Readings = 0;
//    delay(1000);
}


