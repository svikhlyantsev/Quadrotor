
#include <PID_v1.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_10DOF.h>
#include <Adafruit_L3GD20_U.h>
#include <PinChangeInt.h>
#include <Kalman.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
//Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);



float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
float bmpZero;

float temperature;
float elev;

//double smoothElev;
double smoothRoll;
double smoothPitch;
double smoothYaw;

double roll, pitch, yaw;
//PID In out SP cumErr errHold vars

double pitchRateOut, rollRateOut, yawRateOut;
double pitchSetPoint, rollSetPoint, yawSetPoint;

//PID constants

double pitchKP, rollKP, yawKP;
double pitchKI, rollKI, yawKI;
double pitchKD, rollKD, yawKD;

double gyroRoll, gyroPitch, gyroYaw;

double thrust = 204;


double motorSpeed[4] = {0, 0, 0, 0};



//unsigned long pwmTime = 0;

int cali = 0;
sensors_event_t accel_event;
sensors_event_t mag_event;
sensors_event_t bmp_event;
sensors_vec_t   orientation;
sensors_event_t gyro_event;

//unsigned long sensorTime = 0; 
//unsigned long pidTime = 0;
//unsigned long motorSpeedTime = 0;

#define SERVOMIN  204 //0 - 4096
#define SERVOMAX  410

#define RCp1 8
#define RCp2 9
#define RCp3 7
#define RCp4 6
#define RCp7 5

#define servo_flag1 1
#define servo_flag2 2
#define servo_flag3 3
#define servo_flag4 4
#define servo_flag7 7

#define Sensativity 120
#define YawResp 120
#define wrap_180(x) (x < -180 ? x+360 : (x > 180 ? x - 360: x))

volatile uint8_t bUpdatedFlagsShared1;
volatile uint32_t ulServoShared1;
volatile uint16_t unServoShared1;

volatile uint8_t bUpdatedFlagsShared2;
volatile uint32_t ulServoShared2;
volatile uint16_t unServoShared2;

volatile uint8_t bUpdatedFlagsShared3;
volatile uint32_t ulServoShared3;
volatile uint16_t unServoShared3;

volatile uint8_t bUpdatedFlagsShared4;
volatile uint32_t ulServoShared4;
volatile uint16_t unServoShared4;

volatile uint8_t bUpdatedFlagsShared7;
volatile uint32_t ulServoShared7;
volatile uint16_t unServoShared7;

static uint16_t chan1;
static uint16_t chan2;
static uint16_t chan3;
static uint16_t chan4;
static uint16_t chan7;

static double chan1_scaled;
static double chan2_scaled;
static double chan3_scaled;
static double chan4_scaled;




double oriRollOut, oriPitchOut, oriYawOut;
double oriRollSP, oriPitchSP, oriYawSP;
double oriRollKP, oriRollKI, oriRollKD;
double oriPitchKP, oriPitchKI, oriPitchKD;
double oriYawKP, oriYawKI, oriYawKD;

double gyroRollZero = 0;
double gyroPitchZero = 0;
double gyroYawZero = 0;

double oriRollZero = 0;
double oriPitchZero = 0;

PID pitchRatePID(&gyroPitch, &pitchRateOut, &pitchSetPoint ,0,0,0, DIRECT);
PID rollRatePID(&gyroRoll, &rollRateOut, &rollSetPoint,0,0,0, DIRECT);
PID yawRatePID(&gyroYaw, &yawRateOut, &yawSetPoint,0,0,0, DIRECT);

PID oriRollPID(&smoothRoll, &oriRollOut, &oriRollSP,0,0,0, DIRECT);
PID oriPitchPID(&smoothPitch, &oriPitchOut, &oriPitchSP,0,0,0, DIRECT);
PID oriYawPID(&smoothYaw, &oriYawOut, &oriYawSP,0,0,0, DIRECT);

boolean rollRateCheck = false;
boolean pitchRateCheck = false;
boolean yawRateCheck = false;

int rollRateMult = 0;
int pitchRateMult = 0;
int yawRateMult = 0;

double yawTarget = 0;

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
  //if(!bmp.begin())
 // {
  //  /* There was a problem detecting the BMP180 ... check your connections */
  //  Serial.println("Ooops, no BMP180 detected ... Check your wiring!");
   // while(1);
//  }
  if(!gyro.begin())
  {
    /* There was a problem detecting the L3GD20 ... check your connections */
    Serial.println("Ooops, no L3GD20 detected ... Check your wiring!");
    while(1);
  }
}


/////////////IMU Orientation//////////////////
//          X(3)         X(0)
//            \        /
//             \      /
//              \    /
//                
//                |
//                V
//            IMU Y Axis
//            IMU X Axis < --
//            IMU Z Axis: CCW




unsigned long timer;

void setup() {
  
  Serial.begin(115200);

  PCintPort::attachInterrupt(RCp1, RC1, CHANGE);
  PCintPort::attachInterrupt(RCp2, RC2, CHANGE);
  PCintPort::attachInterrupt(RCp3, RC3, CHANGE);
  PCintPort::attachInterrupt(RCp4, RC4, CHANGE);
  PCintPort::attachInterrupt(RCp7, RC7, CHANGE);
  
  pwm.begin();
  pwm.setPWMFreq(50);
  initSensors();
  //bmp.getEvent(&bmp_event);
  //bmp.getTemperature(&temperature);
  //elev = bmp.pressureToAltitude(seaLevelPressure,
                         //bmp_event.pressure,
                         //temperature);
  //bmpZero = elev;
  //elev = elev - bmpZero; 
  //smoothElev = elev;
  gyro.enableAutoRange(true);  
  accel.getEvent(&accel_event);                                      // Orientation Data takes 2.9ms
  dof.accelGetOrientation(&accel_event, &orientation);               //
  mag.getEvent(&mag_event);                                          //  
  dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation);  
  
  //Starting Yaw is target Yaw (prevent spinning on take off)  
  yawTarget = orientation.heading;
  
  
  //Rotational Rate PID Ks
  rollKP  = 0.11;   rollKI  = 1.95;   rollKD  = 0.0037; //vibrations
  pitchKP = 0.11;   pitchKI = 1.95;   pitchKD = 0.0037; //vibrations
  yawKP   = 0.7;   yawKI   = 3.5;   yawKD   = 0.005;///YAW STLL ROTATING A BIT FIX TOMORROW. 
  rollRatePID.SetTunings(rollKP, rollKI, rollKD);
  pitchRatePID.SetTunings(pitchKP, pitchKI, pitchKD);
  yawRatePID.SetTunings(yawKP, yawKI, yawKD);
  
  //Orientation control PID Ks
  oriRollKP  = 1.4;   oriRollKI  = 2.5;   oriRollKD  = 0.025;//Twitchy but working? def needs work
  oriPitchKP = 1.4;   oriPitchKI = 2.5;   oriPitchKD = 0.025;
  oriYawKP   = 4.0 ;   oriYawKI   = 2.75;   oriYawKD   = 0.025;
  oriRollPID.SetTunings(oriRollKP, oriRollKI, oriRollKD);
  oriPitchPID.SetTunings(oriPitchKP, oriPitchKI, oriPitchKD);
  oriYawPID.SetTunings(oriYawKP, oriYawKI, oriYawKD);
  
  //Ini ESCs
  //Enable when testing on rotor
  pwm.setPWM(0, 0, 0);
  pwm.setPWM(1, 0, 0);
  pwm.setPWM(2, 0, 0);
  pwm.setPWM(3, 0, 0);
  //pwm.setPWM(12, 0, 204);
  //pwm.setPWM(13, 0, 204);
  delay(5000); 
  Serial.println("Start Calibration");
  pwm.setPWM( 0, 0, 204);
  pwm.setPWM(1, 0, 204);
  pwm.setPWM(2, 0, 204);
  pwm.setPWM(3, 0, 204);
  //pwm.setPWM(15, 0, 204);
  delay(3000);
  
  //Initalize PID Loops
  rollSetPoint = 0;
  rollRatePID.SetMode(AUTOMATIC);
  rollRatePID.SetSampleTime(14);
  rollRatePID.SetOutputLimits(-400, 400);
  
  pitchSetPoint = 0;
  pitchRatePID.SetMode(AUTOMATIC);
  pitchRatePID.SetSampleTime(14);
  pitchRatePID.SetOutputLimits(-400, 400);
  
  yawSetPoint = 0;
  yawRatePID.SetMode(AUTOMATIC);
  yawRatePID.SetSampleTime(14);
  yawRatePID.SetOutputLimits(-400, 400);
  
  oriRollPID.SetMode(AUTOMATIC);
  oriRollPID.SetSampleTime(14);
  oriRollPID.SetOutputLimits(-180, 180);

  oriPitchPID.SetMode(AUTOMATIC);
  oriPitchPID.SetSampleTime(14);
  oriPitchPID.SetOutputLimits(-180, 180);
  
  oriYawPID.SetMode(AUTOMATIC);
  oriYawPID.SetSampleTime(14);
  oriYawPID.SetOutputLimits(-200, 200);
  
  //Initalize roll/pitch for Sensor Fused calculation
  //roll = orientation.roll;
 // pitch = orientation.pitch;
  
  for(int i = 0; i<1000; i++){
    gyro.getEvent(&gyro_event);
    gyroRollZero += gyro_event.gyro.x;
    gyroPitchZero += gyro_event.gyro.y; 
    gyroYawZero += gyro_event.gyro.z;
  }
  gyroRollZero = gyroRollZero/1000.0;
  gyroPitchZero = gyroPitchZero/1000.0;
  gyroYawZero = gyroYawZero/1000.0;
  
  static unsigned long timelast = 0;
  for(int i = 0; i<1000; i++){
   accel.getEvent(&accel_event);                                      // Orientation Data takes 2.9ms
   dof.accelGetOrientation(&accel_event, &orientation);
   double timeFactor;
    if(timelast == 0){
    roll = orientation.roll; 
    pitch = orientation.pitch;  
    
    }else{
      timeFactor = 1.0/(1000.0/(gyro_event.timestamp - timelast));
      roll = 0.99 * (roll + toDeg((gyro_event.gyro.x - gyroRollZero) * timeFactor) + 0.00) + 0.01 * orientation.roll; 
      pitch = 0.99 * (pitch - toDeg((gyro_event.gyro.y - gyroPitchZero) * timeFactor) - 0.00) + 0.01 * orientation.pitch;  
    }
    oriRollZero += roll; 
    oriPitchZero += pitch;
    timelast = gyro_event.timestamp; 
  }
  oriRollZero = oriRollZero/1000.0;
  oriPitchZero = oriPitchZero/1000.0;
  Serial.println(gyroRollZero, 10);
  Serial.println(gyroPitchZero, 10);
  Serial.println(oriRollZero, 10);
  Serial.println(oriPitchZero, 10);
  roll = orientation.roll;
  pitch = orientation.pitch;
  pwm.setPWM(12, 0, 280);
  pwm.setPWM(13, 0, 314);
  //kalR.setAngle(orientation.roll);
  //kalP.setAngle(orientation.pitch);
  timer = micros();
  Serial.println("done");
  
}

//int counter = 1;
//int counterhold = 1;

//boolean runFlag = true;
boolean firstRun = true;
void loop(){ 
  
  //KALMAN FILTER STUFF
    static Kalman kalmanRoll;
    static Kalman kalmanPitch;
    double kalRV, kalPV;
    if(firstRun){
      
      
      kalmanRoll.setAngle(roll);
      kalmanPitch.setAngle(pitch);
      
      kalmanRoll.setQangle(0.012);
      kalmanPitch.setQangle(0.012);
      
      kalmanRoll.setQbias(0.001);
      kalmanPitch.setQbias(0.001);
      
      kalmanRoll.setRmeasure(5.0);
      kalmanPitch.setRmeasure(5.0);
      

      firstRun = false;
    }
   //Initalize Time step hold for time factor scaling
   static unsigned long timelast = 0;
   
   ///////////CONTROLLER INTERRUPT FOLLOWUP////////////
   if(bUpdatedFlagsShared1){
   noInterrupts();
   chan1 = unServoShared1;
   bUpdatedFlagsShared1 = 0;
   interrupts();
   }
   
   if(bUpdatedFlagsShared2){
   noInterrupts();
   chan2 = unServoShared2;
   bUpdatedFlagsShared2 = 0;
   interrupts();
   }
   
   if(bUpdatedFlagsShared3){
   noInterrupts();
   chan3 = unServoShared3;
   bUpdatedFlagsShared3 = 0;
   interrupts();
   }
   
   if(bUpdatedFlagsShared4){
   noInterrupts();
   chan4 = unServoShared4;
   bUpdatedFlagsShared4 = 0;
   interrupts();
   } 
   //chan7 = 0;
   if(bUpdatedFlagsShared7){
   noInterrupts();
   chan7 = unServoShared7;
   bUpdatedFlagsShared7 = 0;
   interrupts();
   }
    
   ////////////////////////////////////////////
   static boolean motorOffF = false;
   accel.getEvent(&accel_event);                                      // Orientation Data takes 2.9ms
    dof.accelGetOrientation(&accel_event, &orientation);               //
    mag.getEvent(&mag_event);                                          //  
    dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation);    //
    gyro.getEvent(&gyro_event);
    gyroRoll = toDeg(gyro_event.gyro.x - gyroRollZero);// + 1.33; // - (-0.7485)
    gyroPitch = -toDeg(gyro_event.gyro.y - gyroPitchZero);// - 1.03; //- (-0.2693)
    gyroYaw = toDeg(gyro_event.gyro.z - gyroYawZero);
    static double accelRoll, accelPitch, accelYaw;
    double myRoll, myPitch, myYaw;
    
    accelRoll = lpfBasic(accel_event.acceleration.x, 0.9, accelRoll);
    accelPitch = lpfBasic(-accel_event.acceleration.y, 0.9, accelPitch);
    accelYaw = lpfBasic(accel_event.acceleration.z, 0.9, accelYaw);
    myRoll = atan2(-accelPitch, accelYaw)*180.0/PI;
    myPitch = atan2(accelRoll, sqrt(accelPitch*accelPitch + accelYaw*accelYaw))*180.0/PI;
    
    //Offload to RPI?
    //bmp.getEvent(&bmp_event);          //bmp event polling takes 40ms
    //bmp.getTemperature(&temperature);  //
      
    //calculate raw elevation
    /*elev = bmp.pressureToAltitude(seaLevelPressure,
                           bmp_event.pressure,
                           temperature) - bmpZero;
    //Low pass filter elevation to remove noise
    //in need of much work "elevation" fluctates by 1 unit 
  
    smoothElev = lpfBasic(elev, .75, smoothElev);*/
    //elev = smoothElev;  
    //Serial.println(timelast);
    //Serial.println(gyro_event.timestamp);
    double timeFactor;
    static double rollF, pitchF;

    if(timelast == 0){
    roll = orientation.roll; 
    pitch = orientation.pitch;  
    
    }else{
       
      timeFactor = 1.0/(1000.0/(gyro_event.timestamp - timelast));
      roll = 0.99 * (roll + toDeg((gyro_event.gyro.x - gyroRollZero)) * timeFactor) + 0.00 + 0.01 * myRoll; 
      pitch = 0.99* (pitch - toDeg((gyro_event.gyro.y - gyroPitchZero)) * timeFactor) - 0.0 + 0.01 * myPitch; 
      double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
      timer = micros();
      kalRV = kalmanRoll.getAngle(myRoll, gyroRoll, dt)- oriRollZero;; 
      kalPV = kalmanPitch.getAngle(myPitch, gyroPitch, dt)- oriPitchZero;//; 
    }
    //smoothYaw -= toDeg(gyro_event.gyro.z * timeFactor);
    smoothRoll = kalRV - 0.90117;// - (-0.33702);
    smoothPitch = kalPV - 1.481289;// - (0.068589); 
    smoothYaw = orientation.heading;
    timelast = gyro_event.timestamp; 
    pwm.setPWM(13, 0, 314 - map_double(smoothRoll, -90.0, 90.0, -150, 150.0));
    pwm.setPWM(12, 0, 280 - map_double(smoothPitch, -90.0, 90.0, -150.0, 150.0));
   // RC SIGNAL LOST
   if(chan7 < 1500 || chan3 < 1100 + 20){ 
   //Emergency mode
   //Perhaps gradual decent?
   //Serial.println(chan7);
   //Serial.println(chan3);
     thrust = 0;
     pitchSetPoint = 0;
     rollSetPoint = 0;
     yawSetPoint = 0;
     oriRollSP = 0;
     oriPitchSP = 0;
     oriYawSP = 0;
     motorSpeed[0] = 0;
     motorSpeed[2] = 0;
     motorSpeed[1] = 0;
     motorSpeed[3] = 0;
    
     yawTarget = orientation.heading;
     
    if(!motorOffF){
     setMotorSpeed(0, 0, -410, 410, SERVOMIN, SERVOMAX);
     setMotorSpeed(2, 0, -410, 410, SERVOMIN, SERVOMAX);
     setMotorSpeed(1, 0, -410, 410, SERVOMIN, SERVOMAX);
     setMotorSpeed(3, 0, -410, 410, SERVOMIN, SERVOMAX);
     motorOffF = true;
    }
    

     //Serial.println("RC SIGNAL LOST OR TRUST TOO LOW...");
   }else{
      
     motorOffF = false;
     static double chan1F, chan2F, chan3F, chan4F; 
     //READ RC
     ///CHANNEL FLIP BETWEEN 1 AND 2 IS INTENTIONAL
     chan2_scaled = map_double((double)chan1, 1100, 1900, -Sensativity, Sensativity);//roll 
     chan1_scaled = map_double((double)chan2, 1100, 1900, -Sensativity, Sensativity);//pitch 
     chan3_scaled = map_double((double)chan3, 1100, 1900, SERVOMIN, SERVOMAX);//thrust
     chan4_scaled = map_double((double)chan4, 1100, 1900, -YawResp, YawResp);//yaw
     
     //CAP VALUES AND PREVENT OUT OF BOUNDS INPUT
     if(abs(chan1_scaled) > Sensativity){
       chan1_scaled = constrain(chan1_scaled,-Sensativity, Sensativity);
     }
     if(abs(chan2_scaled) > Sensativity){
       chan2_scaled = constrain(chan2_scaled,-Sensativity, Sensativity);
     }
     if(abs(chan3_scaled) > SERVOMAX || abs(chan3_scaled) < SERVOMIN){
       chan3_scaled = 0;
     }
     //if(abs(chan4_scaled) > 180){
     //  chan4_scaled = constrain(chan4_scaled,-180, 180);
     //}
     
     //FILTER ROLL AND PITCH AXIS 
     chan1F = lpfBasic(chan1_scaled, 0.7, chan1F);
     chan2F = lpfBasic(chan2_scaled, 0.7, chan2F);
     chan4F = lpfBasic(chan4_scaled, 0.7, chan4F);
     chan1_scaled = chan1F; 
     chan2_scaled = chan2F;
     chan4_scaled = chan4F;
     
     //ASSGIN RC TO ROTOR FUNCTIONS
     thrust = chan3_scaled;
     /////DEAD ZONE 
     double deadZoneMult = 0.11;//PERCENTAGE OF SENSATIVITY SCALE TO DEAD ZONE
     if(abs(chan2_scaled) > deadZoneMult*Sensativity){ 
       if(chan2_scaled < 0)
         oriPitchSP = chan2_scaled + deadZoneMult*Sensativity;
       else
        oriPitchSP = chan2_scaled - deadZoneMult*Sensativity; 
     }else{
     oriPitchSP = 0;
     }
     if(abs(chan1_scaled) > deadZoneMult*Sensativity){ 
     if(chan1_scaled < 0)
         oriRollSP = chan1_scaled + deadZoneMult*Sensativity;
       else
        oriRollSP = chan1_scaled - deadZoneMult*Sensativity; 
     }else{
     oriRollSP = 0;
     }
     if(abs(chan4_scaled) > deadZoneMult*YawResp){ 
     if(chan4_scaled < 0)
         chan4_scaled = chan4_scaled + deadZoneMult*YawResp;
       else
        chan4_scaled = chan4_scaled - deadZoneMult*YawResp; 
     }else{
     chan4_scaled = 0;
     }
     //oriPitchSP = 0;
     //oriRollSP = 0;
     //oriYawSP = chan4_scaled;
      //Initalize sensor vars
      //Get sensor values (raw)
            
    

      
    //counter++;
    
    
    if(abs(chan4_scaled) > 5){  
     yawSetPoint = chan4_scaled;
     yawTarget = smoothYaw;
   }else{
     oriYawSP = yawTarget;
     yawSetPoint = -oriYawOut;
   }
   
    //yawSetPoint = 0;
    
    oriPitchPID.Compute();
    oriRollPID.Compute();
    
    oriYawPID.Compute();
    
    
    //MODE SELECT ---COMMENT ONE---
    //ORI MODE 
    //rollSetPoint = oriRollOut; 
    //pitchSetPoint = oriPitchOut;
    
    //ACRO MODE
    rollSetPoint = oriRollSP;
    pitchSetPoint = oriPitchSP;
    yawSetPoint = chan4_scaled; 
   
    
    pitchRateCheck = pitchRatePID.Compute();
    rollRateCheck = rollRatePID.Compute();
    yawRateCheck = yawRatePID.Compute();
    

    
 
        
      //rollRateOut = 0;
      //pitchRateOut = 0;
      //yawRateOut = 0;
    if(true){
    /*setMotorSpeed(0, thrust, -410, 410, SERVOMIN, SERVOMAX); 
    setMotorSpeed(1, thrust, -410, 410, SERVOMIN, SERVOMAX); 
    setMotorSpeed(2, thrust, -410, 410, SERVOMIN, SERVOMAX); 
    setMotorSpeed(3, thrust, -410, 410, SERVOMIN, SERVOMAX);  */
    
    
    
    setMotorSpeed(0, thrust + rollRateOut + pitchRateOut - yawRateOut, -410, 410, SERVOMIN, SERVOMAX);
      setMotorSpeed(2, thrust - rollRateOut - pitchRateOut - yawRateOut, -410, 410, SERVOMIN, SERVOMAX);
      setMotorSpeed(1, thrust - rollRateOut + pitchRateOut + yawRateOut, -410, 410, SERVOMIN, SERVOMAX);
      setMotorSpeed(3, thrust + rollRateOut - pitchRateOut + yawRateOut, -410, 410, SERVOMIN, SERVOMAX);
      
      /*Serial.print(",");
      Serial.print(thrust);Serial.print(",");
      Serial.print(oriRollSP);Serial.print(",");
      Serial.print(oriPitchSP);Serial.print(",");
      Serial.print(yawSetPoint);Serial.print(",");*/
      
      Serial.print(smoothRoll);Serial.print(",");
      Serial.print(smoothPitch);Serial.println(",");
      
      /*Serial.print(roll - 0.90117);Serial.print(",");
      Serial.print(pitch - 1.481289);Serial.print(",");
      
      Serial.print(rollRateOut);Serial.print(",");
      Serial.print(pitchRateOut);Serial.print(",");
      Serial.print(yawRateOut);Serial.print(",");
      
      Serial.print(gyroRoll);Serial.print(",");
      Serial.print(gyroPitch);Serial.print(",");
      Serial.print(gyroYaw);Serial.print(",");
      
      Serial.print(accelRoll);Serial.print(",");
      Serial.print(accelPitch);Serial.print(",");
      Serial.print(accelYaw);Serial.println(",");*/
      //Serial.print(",");Serial.print(oriPitchOut);Serial.print(",");Serial.print(oriRollOut);Serial.print(",");Serial.print(pitchRateOut);Serial.print(",");Serial.println(rollRateOut);
      //Serial.print(",");Serial.print(motorSpeed[0]);Serial.print(",");Serial.print(motorSpeed[1]);Serial.print(",");Serial.print(motorSpeed[2]);Serial.print(",");Serial.println(motorSpeed[3]);
      //if(counter > counterhold){
       //counterhold = counter; 
      //}
     // counter = 1;
      
     }
  }
}

double map_double(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double setMotorSpeed(int mNum, double dMspeed, double inMin, double inMax, double mMin, double mMax){
  //motorSpeed[mNum] = motorSpeed[mNum] + dMspeed;
  motorSpeed[mNum] = dMspeed;
  motorSpeed[mNum] = constrain(motorSpeed[mNum], mMin, mMax);
  pwm.setPWM(mNum, 0, motorSpeed[mNum]); 
  return motorSpeed[mNum];
}


double toDeg(double rad){
 return rad*180/PI; 
}

float lpfBasic(float data, float filterVal, float smoothedVal){


  if (filterVal > 1){      // check to make sure param's are within range
    filterVal = .99;
  }
  else if (filterVal <= 0){
    filterVal = 0;
  }

  smoothedVal = (data * (1.0 - filterVal)) + (smoothedVal  *  filterVal);

  return smoothedVal;
}

void RC1(){
  if(digitalRead(RCp1) == HIGH){
   ulServoShared1 = micros(); 
  }else{
   unServoShared1 = (uint16_t)(micros() - ulServoShared1);
   bUpdatedFlagsShared1 |= servo_flag1;
  }
}

void RC2(){
  if(digitalRead(RCp2) == HIGH){
   ulServoShared2 = micros(); 
  }else{
   unServoShared2 = (uint16_t)(micros() - ulServoShared2);
   bUpdatedFlagsShared2 |= servo_flag2;
  }
}

void RC3(){
  if(digitalRead(RCp3) == HIGH){
   ulServoShared3 = micros(); 
  }else{
   unServoShared3 = (uint16_t)(micros() - ulServoShared3);
   bUpdatedFlagsShared3 |= servo_flag3;
  }
}

void RC4(){
  if(digitalRead(RCp4) == HIGH){
   ulServoShared4 = micros(); 
  }else{
   unServoShared4 = (uint16_t)(micros() - ulServoShared4);
   bUpdatedFlagsShared4 |= servo_flag4;
  }
}

void RC7(){
  if(digitalRead(RCp7) == HIGH){
   ulServoShared7 = micros(); 
  }else{
   unServoShared7 = (uint16_t)(micros() - ulServoShared7);
   bUpdatedFlagsShared7 |= servo_flag7;
  }
}

