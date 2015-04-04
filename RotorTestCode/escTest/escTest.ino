#include <Servo.h>

Servo esc;
int throttlePin = 0;
 
void setup()
{
esc.attach(9);
Serial.begin(9600);
}
 
void loop()
{
  int throttle = analogRead(throttlePin);
   Serial.print(throttle, DEC);
   Serial.print("\n");
  throttle = map(throttle, 0, 1023, 0, 179);
  esc.write(throttle);
}
