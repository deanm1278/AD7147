#include <AD7147.h>

#define CS  10 
#define INTERRUPT 11

AD7147 slider = AD7147();

void setup() {
  Serial.begin(9600);
  while(!Serial);
  Serial.println("AD7147 demo");
  
  if(!slider.begin(CS, INTERRUPT)){
    Serial.println("could not communicate with device!!");
    while(1);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  if(slider.update()){
    Serial.println(slider.calculateSiderPosition());
  }
}
