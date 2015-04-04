#include <PinChangeInt.h>
#include <Servo.h>

#define RCp1 6
#define RCp2 7
#define RCp3 8
#define RCp4 9
#define RCp7 5
#define RCp6 4

#define servo_flag1 1
#define servo_flag2 2
#define servo_flag3 3
#define servo_flag4 4
#define servo_flag7 7
#define servo_flag6 6

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

volatile uint8_t bUpdatedFlagsShared6;
volatile uint32_t ulServoShared6;
volatile uint16_t unServoShared6;

static uint16_t chan1;
static uint16_t chan2;
static uint16_t chan3;
static uint16_t chan4;
static uint16_t chan7;
static uint16_t chan6;

void setup(){
 Serial.begin(9600);

PCintPort::attachInterrupt(RCp1, RC1, CHANGE);
PCintPort::attachInterrupt(RCp2, RC2, CHANGE);
PCintPort::attachInterrupt(RCp3, RC3, CHANGE);
PCintPort::attachInterrupt(RCp4, RC4, CHANGE);
PCintPort::attachInterrupt(RCp7, RC7, CHANGE);
PCintPort::attachInterrupt(RCp6, RC6, CHANGE);
}
uint16_t time = 0;
void loop(){
  //static uint16_t unServo;
  //static uint8_t bUpdatedFlags;
  time = micros();
  if(bUpdatedFlagsShared1){
   noInterrupts();
   //bUpdatedFlags = bUpdatedFlagsShared;
   chan1 = unServoShared1;
   bUpdatedFlagsShared1 = 0;
   interrupts();
   }
   
   if(bUpdatedFlagsShared2){
   noInterrupts();
  // bUpdatedFlags = bUpdatedFlagsShared;
   chan2 = unServoShared2;
   bUpdatedFlagsShared2 = 0;
   interrupts();
   }
   
   if(bUpdatedFlagsShared3){
   noInterrupts();
  // bUpdatedFlags = bUpdatedFlagsShared;
   chan3 = unServoShared3;
   bUpdatedFlagsShared3 = 0;
   interrupts();
   }
   
   if(bUpdatedFlagsShared4){
   noInterrupts();
   //bUpdatedFlags = bUpdatedFlagsShared;
   chan4 = unServoShared4;
   bUpdatedFlagsShared4 = 0;
   interrupts();
   }
   if(bUpdatedFlagsShared7){
   noInterrupts();
   //bUpdatedFlags = bUpdatedFlagsShared;
   chan7 = unServoShared7;
   bUpdatedFlagsShared7 = 0;
   interrupts();
   }
   if(bUpdatedFlagsShared6){
   noInterrupts();
   //bUpdatedFlags = bUpdatedFlagsShared;
   chan7 = unServoShared6;
   bUpdatedFlagsShared6 = 0;
   interrupts();
   }
   time = micros() - time;
   if(0){
    Serial.println("RC SIGNAL LOST..."); 
   }else{
   Serial.print(",");Serial.print(chan1);Serial.print(",");Serial.print(chan2);Serial.print(",");Serial.print(chan3);Serial.print(",");Serial.print(chan4);Serial.print(",");Serial.print(chan7);Serial.print(",");Serial.println(chan6);//Serial.println(time); 
   }
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
void RC6(){
  if(digitalRead(RCp6) == HIGH){
   ulServoShared6 = micros(); 
  }else{
   unServoShared6 = (uint16_t)(micros() - ulServoShared6);
   bUpdatedFlagsShared6 |= servo_flag6;
  }
}
