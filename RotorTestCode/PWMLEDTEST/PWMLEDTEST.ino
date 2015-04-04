#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  0 //0 - 4096
#define SERVOMAX  4096

void setup(){
  Serial.begin(9600);
  Serial.println("LED PWM Test");
  pwm.begin();
  pwm.setPWMFreq(60); 
}


void loop(){
  for(uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen+=2){
   pwm.setPWM(0, 0, pulselen);
   Serial.println(pulselen);
   delay(10);
  }
  for(uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen-=2){
   pwm.setPWM(0, 0, pulselen);
   Serial.println(pulselen);
   delay(1);
  }
}
