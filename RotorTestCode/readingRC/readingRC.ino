#include <PinChangeInt.h>
#include <Servo.h>
#define servo_in_pin 7
#define servo_out_pin 9
Servo servo;
#define servo_flag 1
volatile uint8_t bUpdatedFlagsShared;
volatile uint32_t ulServoShared;
volatile uint16_t unServoShared;
void setup(){
 Serial.begin(9600);
servo.attach(servo_out_pin);
PCintPort::attachInterrupt(servo_in_pin, calcServo, CHANGE);
}
void loop(){
  static uint16_t unServo;
  static uint8_t bUpdatedFlags;
  if(bUpdatedFlagsShared){
   noInterrupts();
   bUpdatedFlags = bUpdatedFlagsShared;
   unServo = unServoShared;
   bUpdatedFlagsShared = 0;
   interrupts();
   }
   servo.write(unServo);
}
void calcServo(){
  if(digitalRead(servo_in_pin) == HIGH){
   ulServoShared = micros(); 
  }else{
   unServoShared = (uint16_t)(micros() - ulServoShared);
   bUpdatedFlagsShared |= servo_flag;
  }
}
