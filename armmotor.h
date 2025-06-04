#ifndef _ARMMOTOR_H_
#define _ARMMOTOR_H_
#include <Arduino.h>

class Armmotor {
 public:
  void set(uint8_t pin0, uint8_t pin1, uint8_t pwm);
  void cw(uint8_t level);
  void ccw(uint8_t level);
  void stop();

 private:
  uint8_t _pin0, _pin1, _pwm;
};

#endif
