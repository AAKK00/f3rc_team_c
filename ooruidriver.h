#ifndef _ooruidriver_H_
#define _ooruidriver_H_
#include <Arduino.h>


class ooruidriver{
  public:
    void set(uint8_t motor0, uint8_t motor1);
    void cw(uint8_t level);
    void ccw(uint8_t level);
    void stop();

  private:
    uint8_t _motor0, _motor1, _motor2, _motor3;

};

#endif
