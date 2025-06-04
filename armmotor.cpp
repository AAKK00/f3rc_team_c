#include <Arduino.h>
#include "armmotor.h"

void Armmotor::set(uint8_t pin0, uint8_t pin1, uint8_t pwm) {
  _pin0 = pin0;
  _pin1 = pin1;
  _pwm = pwm;
  pinMode(_pin0, OUTPUT);
  pinMode(_pin1, OUTPUT);
  pinMode(_pwm, OUTPUT);
}

void Armmotor::cw(uint8_t level) {
  digitalWrite(_pin0, HIGH);
  digitalWrite(_pin1, LOW);
  analogWrite(_pwm, level);
}

void Armmotor::ccw(uint8_t level) {
  digitalWrite(_pin0, LOW);
  digitalWrite(_pin1, HIGH);
  analogWrite(_pwm, level);
}

void Armmotor::stop() {
  analogWrite(_pwm, 0);
  digitalWrite(_pin0, LOW);
  digitalWrite(_pin1, LOW);
}
