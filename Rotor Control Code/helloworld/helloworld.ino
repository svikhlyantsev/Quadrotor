// helloworld.ino

#include "helper.h"

void setup() {
Serial.begin(9600);
}

void loop() {
Serial.println(foo()); 
delay(500);
}

