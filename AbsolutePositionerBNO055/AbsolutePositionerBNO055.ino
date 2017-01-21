#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* This driver reads raw data from the BNO055

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

/* Calibration done in New Jersey

Fully calibrated!
--------------------------------
Calibration Results: 
Accelerometer: 20 65454 0 
Gyro: 0 65534 65535 
Mag: 65354 221 65323 
Accel Radius: 1000
Mag Radius: 839

Storing calibration data to EEPROM...
Data stored to EEPROM.

--------------------------------
*/

#define BNO055_SAMPLERATE_DELAY_MS (100)

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

  bno.begin();
}

long lastBNORead = 0;
long nextBNORead = BNO055_SAMPLERATE_DELAY_MS;
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
    sensors_event_t event;
    bno.getEvent(&event);
    Serial.print(F("Orientation: "));
    Serial.print((float)event.orientation.x);
    Serial.print(F(" "));
    Serial.print((float)event.orientation.y);
    Serial.print(F(" "));
    Serial.print((float)event.orientation.z);
    Serial.println(F(""));

    lastBNORead = millis();
  }
}

void setup(void)
{
  Serial.begin(9600);
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  StartBNO();
}

void loop(void)
{
  ReadBNO();
}