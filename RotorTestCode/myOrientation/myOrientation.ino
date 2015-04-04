#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_10DOF.h>
#include <Adafruit_L3GD20_U.h>

/* Assign a unique ID to the sensors */
Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);

/* Update this with the correct SLP for accurate altitude measurements */
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

/**************************************************************************/
/*!
    @brief  Initialises all the sensors used by this example
*/
/**************************************************************************/
void initSensors()
{
  if(!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP180 ... check your connections */
    Serial.println("Ooops, no BMP180 detected ... Check your wiring!");
    while(1);
  }
  if(!gyro.begin())
  {
    /* There was a problem detecting the L3GD20 ... check your connections */
    Serial.println("Ooops, no L3GD20 detected ... Check your wiring!");
    while(1);
  }
}

/**************************************************************************/
/*!

*/
/**************************************************************************/



void displaySensorDetails(void)
{
  sensor_t sensor;
  gyro.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" rad/s");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" rad/s");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" rad/s");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

double pitch,roll,yaw;
sensors_event_t accel_event;
sensors_event_t mag_event;
sensors_event_t bmp_event;
sensors_vec_t   orientation;
sensors_event_t gyro_event; 

double gyroZeroX, gyroZeroY, gyroZeroZ;

double sums[6] = {0,0,0,0,0,0};

void setup(void)
{
  Serial.begin(115200);

  gyro.enableAutoRange(true);
  /* Initialise the sensors */
  initSensors();
  displaySensorDetails();
  
  
  for(int i = 0; i <300; i++){
  accel.getEvent(&accel_event);
  dof.accelGetOrientation(&accel_event, &orientation);
  mag.getEvent(&mag_event);
  dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation);
 }
  pitch = orientation.pitch;
  roll = orientation.roll;
  Serial.println(roll);
  yaw = orientation.heading;
  
  gyroZeroX = 0.0;
  gyroZeroY = 0.0;
  gyroZeroZ = 0.0;
  /*static unsigned long timelast = 0;
  for(int i = 0; i < 500; i++){
    gyro.getEvent(&gyro_event);
    double timeFactor = 1.0/(1000.0/(gyro_event.timestamp - timelast));
    gyroZeroX += toDeg(gyro_event.gyro.x * timeFactor); 
    gyroZeroY += toDeg(gyro_event.gyro.y  * timeFactor); 
    gyroZeroZ += toDeg(gyro_event.gyro.z * timeFactor);
    //Serial.print(gyroZeroX);Serial.print(",");Serial.print(gyroZeroY);Serial.print(",");Serial.println(gyroZeroZ);
    timelast = gyro_event.timestamp;
    delay(14);
  }
  gyroZeroX = gyroZeroX/500.0; 
  gyroZeroY = gyroZeroY/500.0; 
  gyroZeroZ = gyroZeroZ/500.0;
  Serial.print(gyroZeroX);Serial.print(",");Serial.print(gyroZeroY);Serial.print(",");Serial.println(gyroZeroZ);*/
  
}



void loop()
{
 for(int i = 0; i<1700; i++){
 static unsigned long timelast = 0;
 accel.getEvent(&accel_event);
 dof.accelGetOrientation(&accel_event, &orientation);
 mag.getEvent(&mag_event);
 dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation);
 gyro.getEvent(&gyro_event);
 //Serial.println(timelast);
//Serial.println(gyro_event.timestamp);
 double timeFactor;
    if(timelast == 0)
      timeFactor = 1.0/(1000.0/(23));
    else
      timeFactor = 1.0/(1000.0/(gyro_event.timestamp - timelast));
      //Serial.println(roll);
 roll = (0.98 * ( roll + toDeg((gyro_event.gyro.x - 0)   * timeFactor)) + 0.02 * orientation.roll); 
 pitch = (0.98 * (pitch - toDeg((gyro_event.gyro.y + 0)  * timeFactor)) + 0.02 * orientation.pitch); 
 yaw -= toDeg(gyro_event.gyro.z * timeFactor) - gyroZeroZ;
 
 timelast = gyro_event.timestamp;
 
Serial.print(",");Serial.print(gyro_event.gyro.x);
Serial.print(",");Serial.print(gyro_event.gyro.y);
Serial.print(",");Serial.print(gyro_event.gyro.z);
Serial.print(",");Serial.print(orientation.roll);
Serial.print(",");Serial.print(roll + 1.85);
Serial.print(",");Serial.print(orientation.pitch);
Serial.print(",");Serial.print(pitch - 1.44);
Serial.print(",");Serial.print(orientation.heading);
Serial.print(",");Serial.println(yaw);
sums[1] += gyro_event.gyro.x;
sums[2] += gyro_event.gyro.y;
sums[3] += gyro_event.gyro.z;
sums[4] += roll;
sums[5] += pitch;

delay(14);
 }
 Serial.println("Averages:");
 Serial.print(sums[1]/1700);Serial.print(",");
 Serial.print(sums[2]/1700);Serial.print(",");
 Serial.print(sums[3]/1700);Serial.print(",");
 Serial.print(sums[4]/1700);Serial.print(",");
 Serial.println(sums[5]/1700);
 Serial.println("Done with sums");
 while(true){};
}



double toDeg(double rad){
 return rad*180/PI; 
}


