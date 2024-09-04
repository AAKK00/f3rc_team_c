#ifndef _motormovement_H_
#define _motormovement_H_
#include <Arduino.h>
#include "ooruidriver.h"


class motormovement{
  public:
    void set(ooruidriver motor0, ooruidriver motor1, ooruidriver motor2, ooruidriver motor3);
    void forward(uint8_t level);
    void back(uint8_t level);
    void cw(uint8_t level);
    void ccw(uint8_t level);
    void stop();
    void right(uint8_t level);
    void left(uint8_t level);
    void rightforward(uint8_t level);
    void leftforward(uint8_t level);
    void rightback(uint8_t level);
    void leftback(uint8_t level);

  private:
    ooruidriver _motor0, _motor1, _motor2, _motor3;

};

#endif
