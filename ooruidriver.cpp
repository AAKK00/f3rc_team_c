#include <Arduino.h>
#include "ooruidriver.h"

void ooruidriver::set(uint8_t motor0, uint8_t motor1){
  _motor0 = motor0;
  _motor1 = motor1;

  pinMode(_motor0, OUTPUT);
  pinMode(_motor1, OUTPUT);
}

void ooruidriver::cw(uint8_t level){
  analogWrite(_motor0, level);
  analogWrite(_motor1, 0);
}

void ooruidriver::ccw(uint8_t level){
  analogWrite(_motor0, 0);
  analogWrite(_motor1, level);
}

void ooruidriver::stop(){
  analogWrite(_motor0, 0);
  analogWrite(_motor1, 0);
}
