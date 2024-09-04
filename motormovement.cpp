#include <Arduino.h>
#include "ooruidriver.h"
#include "motormovement.h"

void motormovement::set(ooruidriver motor0, ooruidriver motor1, ooruidriver motor2, ooruidriver motor3){
  _motor0 = motor0;
  _motor1 = motor1;
  _motor2 = motor2;
  _motor3 = motor3;
}

void motormovement::forward(uint8_t level){
  _motor0.cw(level);
  _motor1.cw(level);
  _motor2.ccw(level);
  _motor3.ccw(level);
}

void motormovement::back(uint8_t level){
  _motor0.ccw(level);
  _motor1.ccw(level);
  _motor2.cw(level);
  _motor3.cw(level);
}

void motormovement::cw(uint8_t level){
  _motor0.cw(level);
  _motor1.cw(level);
  _motor2.cw(level);
  _motor3.cw(level);
}

void motormovement::ccw(uint8_t level){
  _motor0.ccw(level);
  _motor1.ccw(level);
  _motor2.ccw(level);
  _motor3.ccw(level);
}

void motormovement::stop(){
  _motor0.stop();
  _motor1.stop();
  _motor2.stop();
  _motor3.stop();
}

void motormovement::right(uint8_t level){
  _motor0.ccw(level);
  _motor1.cw(level);
  _motor2.cw(level);
  _motor3.ccw(level);
}

void motormovement::left(uint8_t level){
  _motor0.cw(level);
  _motor1.ccw(level);
  _motor2.ccw(level);
  _motor3.cw(level);
}

void motormovement::rightforward(uint8_t level){
  _motor1.cw(level);
  _motor3.ccw(level);
}

void motormovement::leftforward(uint8_t level){
  _motor0.cw(level);
  _motor2.ccw(level);
}

void motormovement::rightback(uint8_t level){
  _motor1.ccw(level);
  _motor3.cw(level);
}

void motormovement::leftback(uint8_t level){
  _motor0.ccw(level);
  _motor2.cw(level);
}
