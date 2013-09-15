#include <Wire.h>        // I2C library
#include <ADXL345.h>     // Accelerometer library
#include <L3G4200D.h>    // Gyroscope library
#include <HMC5883L.h>    // Magnetrometer library
#include <AHRS.h>        // AHRS orientation filter library
#include <avr/eeprom.h>  // EEPROM library (AVR)
#include <EEPROM.h>      // EEPROM library (Arduino)
//#include <FastMath.h>

////////////// OUTPUT OPTION /////////////////
#define BAUD_RATE                       115200
#define OUTPUT_DATA_INTERVAL            10    // 10ms, 100Hz update rate
// Output format
#define OUTPUT_FORMAT_TEXT              0     // Output as text
#define OUTPUT_FORMAT_BINARY            1     // Output as binary
// Output mode
#define OUTPUT_MODE_ANGLE               0     // Output yaw, pitch, roll in degress
#define OUTPUT_MODE_CALIBRATE_ACCEL     1     // Output accel min/max as text
#define OUTPUT_MODE_CALIBRATE_GYRO      2     // Output gyro min/max as text
#define OUTPUT_MODE_CALIBRATE_MGN       3     // Output mgn min/max as text
#define OUTPUT_MODE_SENSORS_CALIB       4     // Output calibrated sensor values for all 9 axes
#define OUTPUT_MODE_SENSORS_RAW         5     // Output uncalibrated (raw) sensor values for all 9 axes
#define OUTPUT_MODE_SENSORS_MIN_MAX     6     // Output sensor min and max values
#define OUTPUT_MODE_NOTHING             7     // Output nothing
// Default output format and mode
int outputFormat = OUTPUT_FORMAT_TEXT;
int outputMode = OUTPUT_MODE_ANGLE;
////////////////////////////////////////

////////////// SENSOR CALIBRATION ////////////
// Magnetometer (extended calibration)
// Uncommend to use extended magnetometer calibration (compensates hard & soft iron errors)
//#define CALIBRATION__MAGN_USE_EXTENDED true
//const float magn_ellipsoid_center[3] = {0, 0, 0};
//const float magn_ellipsoid_transform[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

/* Sensor calibration offset values and scale
#define ACCEL_X_SCALE (GRAVITY / (ACCEL_X_MAX - ACCEL_X_OFFSET))
#define ACCEL_Y_SCALE (GRAVITY / (ACCEL_Y_MAX - ACCEL_Y_OFFSET))
#define ACCEL_Z_SCALE (GRAVITY / (ACCEL_Z_MAX - ACCEL_Z_OFFSET))
#define MAGN_X_SCALE (100.0f / (MAGN_X_MAX - MAGN_X_OFFSET))
#define MAGN_Y_SCALE (100.0f / (MAGN_Y_MAX - MAGN_Y_OFFSET))
#define MAGN_Z_SCALE (100.0f / (MAGN_Z_MAX - MAGN_Z_OFFSET))*/
//////////////////////////////////////////////

// Other define
#define GYRO_SCALED_RAD  0.000305432  // Calculate the scaled gyro readings in radians per second
/*
  65536 full scale
  17.5mili-degree/s per digit
  GYRO_SCALED_RAD = 0.0175 * 3.141592654 / 180 = 0.000305432
*/
#define RAW_DATA         0
#define CALIBRATED_DATA  1
#define LED_STATUS_PIN   13

// Sensor objects
ADXL345 accel;
L3G4200D gyro;
HMC5883L mgn;
AHRS ahrs;
// Sensor variables
int accelXYZ[3], gyroXYZ[3], mgnXYZ[3];
float crtAccelXYZ[3], crtGyroXYZ[3], crtMgnXYZ[3];
struct sensorConfig
{
  int accelMin[3];
  int accelMax[3];
  float gyroAverage[3];
  int mgnMin[3];
  int mgnMax[3];
} sensor, sensorDisplay;
long gyroSum[3] = {0}, gyroNumSample = 0;
float accelOffset[3], mgnOffset[3];
boolean hasAccel, hasGyro, hasMgn;
// Euler angles
float yaw, pitch, roll;
// Time stamp
unsigned long timestamp = millis();
// Other variales
boolean sensorCalibrated = false;
int ledStatusFlag = 0;

void setup()
{
  Serial.begin(BAUD_RATE);
  Wire.begin();
  // Accelerometer init
  accel.Init(ADXL345_ADDR_ALT_LOW);
  if (accel.errorCode == ADXL345_NO_ERROR)
  {
    accel.SetRate(ADXL345_BW_400);
    accel.SetRange(ADXL345_RANGE_4G);
    accel.SetFullResolution(true);
    hasAccel = true;
    //accel.PrintAllRegister();
    //Serial.print("Accel Device ID: 0x");
    //Serial.println(accel.GetDevID(), HEX);
  }
  else
  {
    hasAccel = false;
    Serial.println("No Accelerometer");
  }
  
  // Gryoscope init
  gyro.Init(L3G4200D_ADDR_ALT_HIGH);
  if (gyro.errorCode == L3G4200D_NO_ERROR)
  {
    gyro.SetRate(L3G4200D_RATE_400);
    gyro.SetBandwidth(L3G4200D_BANDWIDTH_1);
    gyro.SetHighPassFilterMode(L3G4200D_NORMAL_RESET_MODE);
    gyro.SetResolution(500);
    gyro.EnableHighPassFilter(true);
    gyro.SetFIFOMode(L3G4200D_BYPASS_MODE);
    hasGyro = true;
    //gyro.PrintAllRegister();
    //Serial.print("Gyro Device ID: 0x");
    //Serial.println(gyro.GetDevID(), HEX);
  }
  else
  {
    hasGyro = false;
    Serial.println("No Gyroscope");
  }
  
  // Magnetrometer init
  mgn.Init(HMC5883L_ADDR);
  if (mgn.errorCode == HMC5883L_NO_ERROR)
  {
    mgn.SetMeasureMode(HMC5883L_NORMAL_MEASURE);
    mgn.SetRate(HMC5883_ODR_15);
    mgn.SetSampleAverage(HMC5883L_8_SAMPLE);
    mgn.SetGain(HMC5883L_GAIN_1_3);
    mgn.SetOutputMode(HMC5883L_CONTINUOUS_MODE);
    hasMgn = true;
    //mgn.PrintAllRegister();
    //Serial.print("Mgn Device ID: 0x");
    //Serial.println(mgn.GetDevID(), HEX);
  }
  else
  {
    hasMgn = false;
    Serial.println("No Magnetometer");
  }
  
  // Set pin mode
  pinMode(LED_STATUS_PIN, OUTPUT);
  // EEPROM checking
  for (int i=0; i<3; i++)
  {
    sensorDisplay.accelMin[i] = 0;
    sensorDisplay.accelMax[i] = 0;
    sensorDisplay.gyroAverage[i] = 0;
    sensorDisplay.mgnMin[i] = 0;
    sensorDisplay.mgnMax[i] = 0;
  }
  if (EEPROM.read(0) != 50 && EEPROM.read(1) != 100 && EEPROM.read(2) != 150)
  {
    sensorCalibrated = false;
    Serial.println("No stored data");
    for (int i=0; i<3; i++)
    {
      sensor.accelMin[i] = 0;
      sensor.accelMax[i] = 0;
      sensor.gyroAverage[i] = 0;
      sensor.mgnMin[i] = 0;
      sensor.mgnMax[i] = 0;
    }
  }
  else
  {
    sensorCalibrated = true;
    Serial.println("Reading data from EEPROM");
    eeprom_read_block((void*)&sensor, (void*)4, sizeof(sensor));
  }
}

void loop()
{
  // Read incoming control messages
  if (Serial.available() >= 2)
  {
    if (Serial.read() == '#')    // Header of the message
    {
      char command = Serial.read();
      if (command == 's')      // Synch request
      {
        byte id[2];
        id[0] = Serial.read();
        id[1] = Serial.read();
        Serial.print("#SYNCH");
        Serial.write(id, 2);
        Serial.println();
      }
      else if (command == 'o')  // Set output mode
      {
        char id = Serial.read();
        if (id == 't')
        {
          outputFormat = OUTPUT_FORMAT_TEXT;
          outputMode = OUTPUT_MODE_ANGLE;
        }
        else if (id == 'b')
        {
          outputFormat = OUTPUT_FORMAT_BINARY;
          outputMode = OUTPUT_MODE_ANGLE;
        }
        else if (id == 'r')
        {
          outputMode = OUTPUT_MODE_SENSORS_RAW;
        }
        else if (id == 'f')
        {
          outputMode = OUTPUT_MODE_SENSORS_CALIB;
        }
        else if (id == 'c')
        {
          char sensorType = Serial.read();
          if (sensorType == 'a')
          {
            for (int i=0; i<3; i++)
            {
              sensorDisplay.accelMin[i] = 0;
              sensorDisplay.accelMax[i] = 0;
            }
            outputMode = OUTPUT_MODE_CALIBRATE_ACCEL;
          }
          else if (sensorType == 'g')
          {
            for (int i=0; i<3; i++)
            {
              sensorDisplay.gyroAverage[i] = 0;
              gyroSum[i] = 0;
            }
            gyroNumSample = 0;
            outputMode = OUTPUT_MODE_CALIBRATE_GYRO;
          }
          else if (sensorType == 'm')
          {
            for (int i=0; i<3; i++)
            {
              sensorDisplay.mgnMin[i] = 0;
              sensorDisplay.mgnMax[i] = 0;
            }
            outputMode = OUTPUT_MODE_CALIBRATE_MGN;
          }
        }
      }
      else if (command == 'e')    // Save data to EEPROM
      {
        if (outputMode == OUTPUT_MODE_CALIBRATE_ACCEL)
        {  
          for (int i=0; i<3; i++)
          {
            // Copy new data to "sensor"
            sensor.accelMin[i] = sensorDisplay.accelMin[i];
            sensor.accelMax[i] = sensorDisplay.accelMax[i];
          }
          // To indicate data had been stored in EEPROM
          EEPROM.write(0, 50);
          eeprom_write_block((const void*)&sensor, (void*)4, sizeof(sensor));
        }
        else if (outputMode == OUTPUT_MODE_CALIBRATE_GYRO)
        {
          for (int i=0; i<3; i++)
          {
            sensor.gyroAverage[i] = sensorDisplay.gyroAverage[i];
          }
          EEPROM.write(1, 100);
          eeprom_write_block((const void*)&sensor, (void*)4, sizeof(sensor));
        }
        else if (outputMode == OUTPUT_MODE_CALIBRATE_MGN)
        {
          for (int i=0; i<3; i++)
          {
            sensor.mgnMin[i] = sensorDisplay.mgnMin[i];
            sensor.mgnMax[i] = sensorDisplay.mgnMax[i];        
          }
          EEPROM.write(2, 150);
          eeprom_write_block((const void*)&sensor, (void*)4, sizeof(sensor));
        }
      }
      else if (command == 'd')
      {
        outputMode = OUTPUT_MODE_SENSORS_MIN_MAX;
      }
    }
  }
  
  // For oscilloscope to check the update rate
  //digitalWrite(13, HIGH);
  //digitalWrite(13, LOW);
  // LED indication
  ledIndication();
  // Read sensors' data
  accel.ReadAccel(accelXYZ);
  gyro.ReadGyro(gyroXYZ);
  mgn.ReadMgn(mgnXYZ);
  // Compensate sensor error
  for (int i=0; i<3; i++)
  {
    // Calculate accelerometer and magnetrometer offset
    accelOffset[i] = (sensor.accelMin[i] + sensor.accelMax[i]) / 2.0f;
    mgnOffset[i] = (sensor.mgnMin[i] + sensor.mgnMax[i]) / 2.0f;
    // Calculate correct sensor value
    crtAccelXYZ[i] = accelXYZ[i] - accelOffset[i];
    crtGyroXYZ[i] = (gyroXYZ[i] - sensor.gyroAverage[i]) * GYRO_SCALED_RAD;
    crtMgnXYZ[i] = mgnXYZ[i] - mgnOffset[i];
  }
  ahrs.AHRSupdate(crtGyroXYZ[0], crtGyroXYZ[1], crtGyroXYZ[2],
                  crtAccelXYZ[0], crtAccelXYZ[1], crtAccelXYZ[2],
                  crtMgnXYZ[0], crtMgnXYZ[1], crtMgnXYZ[2],
                  &yaw, &pitch, &roll);
  
  // Check output mode
  if (outputMode == OUTPUT_MODE_ANGLE)
  {
    if (outputFormat == OUTPUT_FORMAT_TEXT)
      printYPRtext();
    else if (outputFormat == OUTPUT_FORMAT_BINARY)
      printYPRbin();
  }
  else if (outputMode == OUTPUT_MODE_CALIBRATE_ACCEL)
  {
    Serial.print("Accel xyz min/max: ");
    for (int i=0; i<3; i++)
    {
      if (accelXYZ[i] < sensorDisplay.accelMin[i])
        sensorDisplay.accelMin[i] = accelXYZ[i];
      else if (accelXYZ[i] > sensorDisplay.accelMax[i])
        sensorDisplay.accelMax[i] = accelXYZ[i];
      
      Serial.print(sensorDisplay.accelMin[i]);
      Serial.print("/");
      Serial.print(sensorDisplay.accelMax[i]);
      if (i < 2)
        Serial.print(",");
     else
       Serial.println();
    }
  }
  else if (outputMode == OUTPUT_MODE_CALIBRATE_GYRO)
  {
    gyroNumSample++;
    
    Serial.print("Gyro xyz current/average: ");
    for (int i=0; i<3; i++)
    {
      gyroSum[i] += gyroXYZ[i];
      sensorDisplay.gyroAverage[i] = gyroSum[i] / (float)gyroNumSample;
      Serial.print(gyroXYZ[i]);
      Serial.print("/");
      Serial.print(sensorDisplay.gyroAverage[i]);
      if (i < 2)
        Serial.print(",");
      else
        Serial.println();
    }
  }
  else if (outputMode == OUTPUT_MODE_CALIBRATE_MGN)
  {
    Serial.print("Mgn xyz min/max: ");
    for (int i=0; i<3; i++)
    {
      if (mgnXYZ[i] < sensorDisplay.mgnMin[i])
        sensorDisplay.mgnMin[i] = mgnXYZ[i];
      else if (mgnXYZ[i] > sensorDisplay.mgnMax[i])
        sensorDisplay.mgnMax[i] = mgnXYZ[i];
      
      Serial.print(sensorDisplay.mgnMin[i]);
      Serial.print("/");
      Serial.print(sensorDisplay.mgnMax[i]);
      if (i < 2)
        Serial.print(",");
      else
        Serial.println();
    }
  }
  else if (outputMode == OUTPUT_MODE_SENSORS_CALIB)
  {
    Serial.print("[C]");
    printAccel(CALIBRATED_DATA);
    printGyro(CALIBRATED_DATA);
    printMgn(CALIBRATED_DATA);
    Serial.println();
  }
  else if (outputMode == OUTPUT_MODE_SENSORS_RAW)
  {
    Serial.print("[R]");
    printAccel(RAW_DATA);
    printGyro(RAW_DATA);
    printMgn(RAW_DATA);
    Serial.println();
  }
  else if (outputMode == OUTPUT_MODE_SENSORS_MIN_MAX)
  {
    outputMode = OUTPUT_MODE_NOTHING;
    Serial.print("Accel min/max: ");
    for (int i=0; i<3; i++)
    {
      Serial.print(sensor.accelMin[i]);
      Serial.print("/");
      Serial.print(sensor.accelMax[i]);
      if (i < 2)
        Serial.print(",");
      else
        Serial.println();
    }
    Serial.print("Gyro average: ");
    for (int i=0; i<3; i++)
    {
      Serial.print(sensor.gyroAverage[i]);
      if (i < 2)
        Serial.print(",");
      else
        Serial.println();
    }
    Serial.print("Mgn min/max: ");
    for (int i=0; i<3; i++)
    {
      Serial.print(sensor.mgnMin[i]);
      Serial.print("/");
      Serial.print(sensor.mgnMax[i]);
      if (i < 2)
        Serial.print(",");
      else
        Serial.println();
    }
  }
  else if (outputMode == OUTPUT_MODE_NOTHING)
  {}  // Do nothing
  
  while (millis() - timestamp < OUTPUT_DATA_INTERVAL) {}  // limit the update rate
  timestamp = millis();
}

void ledIndication()
{
  if (sensorCalibrated)
  {
    digitalWrite(LED_STATUS_PIN, HIGH);
  }
  else
  {
    if (millis() % 200 == 0)
    {
      if (ledStatusFlag == 0)
      {
        digitalWrite(LED_STATUS_PIN, HIGH);
        ledStatusFlag = 1;
      }
      else
      {
        digitalWrite(LED_STATUS_PIN, LOW);
        ledStatusFlag = 0;
      }
    }
  }
}

void printYPRtext()
{
  Serial.print("#YPR=");
  Serial.print(yaw);
  Serial.print(",");
  Serial.print(pitch);
  Serial.print(",");
  Serial.print(roll);
  Serial.println();
}

void printYPRbin()
{
  float ypr[3] = {(int)yaw, (int)pitch, (int)roll};
  Serial.write((byte*) ypr, 12);
}

void printAccel(int calibrated)
{
  if (calibrated)
  {
    Serial.print("A: ");
    Serial.print(crtAccelXYZ[0]);
    Serial.print(",");
    Serial.print(crtAccelXYZ[1]);
    Serial.print(",");
    Serial.print(crtAccelXYZ[2]);
    Serial.print("; ");
  }
  else
  {
    Serial.print("A: ");
    Serial.print(accelXYZ[0]);
    Serial.print(",");
    Serial.print(accelXYZ[1]);
    Serial.print(",");
    Serial.print(accelXYZ[2]);
    Serial.print("; ");
  }
}

void printGyro(int calibrated)
{  
  if (calibrated)
  {
    Serial.print("G: ");
    Serial.print(crtGyroXYZ[0]);
    Serial.print(",");
    Serial.print(crtGyroXYZ[1]);
    Serial.print(",");
    Serial.print(crtGyroXYZ[2]);
    Serial.print("; ");
  }
  else
  {
    Serial.print("G: ");
    Serial.print(gyroXYZ[0]);
    Serial.print(",");
    Serial.print(gyroXYZ[1]);
    Serial.print(",");
    Serial.print(gyroXYZ[2]);
    Serial.print("; ");
  }
}

void printMgn(int calibrated)
{
  if (calibrated)
  {
    Serial.print("M: ");
    Serial.print(crtMgnXYZ[0]);
    Serial.print(",");
    Serial.print(crtMgnXYZ[1]);
    Serial.print(",");
    Serial.print(crtMgnXYZ[2]);
    Serial.print("; ");
  }
  else
  {
    Serial.print("M: ");
    Serial.print(mgnXYZ[0]);
    Serial.print(",");
    Serial.print(mgnXYZ[1]);
    Serial.print(",");
    Serial.print(mgnXYZ[2]);
    Serial.print("; ");
  }
}
