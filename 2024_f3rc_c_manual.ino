#include <Arduino.h>
#include <math.h>

#include <XboxSeriesXControllerESP32_asukiaaa.hpp>

#include "ooruidriver.h"
#include "motormovement.h"
#include "armmotor.h"


#define motor0_0 13
#define motor0_1 12

#define motor1_0 27
#define motor1_1 14

#define motor2_0 33
#define motor2_1 32

#define motor3_0 25
#define motor3_1 26


#define armmotor0_0 16
#define armmotor0_1 17
#define armmotor0_pwm 0

#define armmotor1_0 22
#define armmotor1_1 23
#define armmotor1_pwm 4

#define push 26

#define arm_control_0 34
#define arm_control_1 35
#define reverse 39



XboxSeriesXControllerESP32_asukiaaa::Core xboxController;

ooruidriver motor0, motor1, motor2, motor3;
motormovement Drive;

Armmotor arm0, arm1;


void setup() {
  Serial.begin(115200);
  motor0.set(motor0_0, motor0_1);
  motor1.set(motor1_0, motor1_1);
  motor2.set(motor2_0, motor2_1);
  motor3.set(motor3_0, motor3_1);

  Drive.set(motor0, motor1, motor2, motor3);

  arm0.set(armmotor0_0, armmotor0_1, armmotor0_pwm);
  arm1.set(armmotor1_0, armmotor1_1, armmotor1_pwm);

  pinMode(push, OUTPUT);
  digitalWrite(push, LOW);

  pinMode(arm_control_0, INPUT);
  pinMode(arm_control_1, INPUT);
  pinMode(reverse, INPUT);

  xboxController.begin();
}



void loop() {

  xboxController.onLoop();
  if (xboxController.isConnected()) {
    if (xboxController.isWaitingForFirstNotification()) {
      Serial.println("waiting for first notification");
    } else {
      Serial.println("Address: " + xboxController.buildDeviceAddressStr());
      //Serial.print(xboxController.xboxNotif.toString());
      unsigned long receivedAt = xboxController.getReceiveNotificationAt();
      uint16_t joystickMax = XboxControllerNotificationParser::maxJoy;
      
      float joylhori_rate = (float)xboxController.xboxNotif.joyLHori / joystickMax;
      
      
      float joylvert_rate = (float)xboxController.xboxNotif.joyLVert / joystickMax;
      

      int level = 100;
      joylhori_rate = (joylhori_rate - 0.5) * 2.0;
      joylvert_rate = -(joylvert_rate - 0.5) * 2.0;
      Serial.print("joyLHori rate: ");
      Serial.println(joylhori_rate);
      Serial.print("joyLVert rate: ");
      Serial.println(joylvert_rate);
      float rL = sqrt(pow(joylhori_rate, 2) + pow(joylvert_rate, 2));
      float theta = atan2(joylvert_rate, joylhori_rate);
      Serial.print("thata:");
      Serial.println(theta);
      if(rL >= 1.0){
        rL = 1.0;
      }
      level = level * rL;
      Serial.print("rL:");
      Serial.println(rL);

      

      if (rL >= 0.08){
        
        if (1 * M_PI / 8 <= theta && theta < 3 * M_PI / 8) {
          Drive.rightforward(level);
          Serial.println("rightforward");
        } else if (3 * M_PI / 8 <= theta && theta < 5 * M_PI / 8) {
          Drive.forward(level);
          Serial.println("forward");
        } else if (5 * M_PI / 8 <= theta && theta < 7 * M_PI / 8) {
          Drive.leftforward(level);
          Serial.println("leftforward");
        } else if (-7 * M_PI / 8 <= theta && theta < -5 * M_PI / 8) {
          Drive.leftback(level);
          Serial.println("leftback");
        } else if (-5 * M_PI / 8 <= theta && theta < -3 * M_PI / 8) {
          Drive.back(level);
          Serial.println("back");
        } else if (-3 * M_PI / 8 <= theta && theta < -1 * M_PI / 8) {
          Drive.rightback(level);
          Serial.println("rightback");
        } else if (-1 * M_PI / 8 <= theta && theta < 1 * M_PI / 8) {
          Drive.right(level);
          Serial.println("right");
        } else {
          Drive.left(level);
          Serial.println("left");
        }
      } else {
        Drive.stop();
      }

      Serial.print("joyRHori rate: ");
      float joyrhori_rate = (float)xboxController.xboxNotif.joyRHori / joystickMax;
      Serial.println(joyrhori_rate);
      Serial.print("joyRVert rate: ");
      float joyrvert_rate = (float)xboxController.xboxNotif.joyRVert / joystickMax;
      Serial.println(joyrvert_rate);

      int rotate = 250;
      joyrhori_rate = (joyrhori_rate - 0.5) * 2.0;
      joyrvert_rate = -(joyrvert_rate - 0.5) * 2.0;
      float rR = sqrt(pow(joyrhori_rate,2) + pow(joyrvert_rate,2));
      float phi = atan2(joyrvert_rate, joyrhori_rate);
      Serial.print("phi:");
      Serial.println(phi);
      if(rR >= 1.0){
        rR = 1.0;
      }
      rotate = rotate * rR;

      if (rR >= 0.08) {
        if (-1 * M_PI / 4 <= phi && phi < 1 * M_PI / 4) {
          Drive.cw(rotate);
          Serial.println("cw");
        }
        else if (3 * M_PI / 4 <= phi || phi <= -3 * M_PI / 4) {
          Drive.ccw(rotate);
          Serial.println("ccw");
        }
      }


      if(xboxController.xboxNotif.btnA == 1) {
        arm0.ccw(xboxController.xboxNotif.trigRT / 4);
        arm1.cw(xboxController.xboxNotif.trigLT / 4);
      } else {
        arm0.cw(xboxController.xboxNotif.trigRT / 4);
        arm1.ccw(xboxController.xboxNotif.trigLT / 4);
      }

      if(xboxController.xboxNotif.btnB == 1) {
        digitalWrite(push, HIGH);
      } else {
        digitalWrite(push, LOW);
      }


      Serial.println("battery " + String(xboxController.battery) + "%");
      Serial.println("received at " + String(receivedAt));
    }
  } else {
    Serial.println("not connected");
    if (xboxController.getCountFailedConnection() > 2) {
      ESP.restart();
    }
  }
  delay(100);


  if (!digitalRead(reverse)) {
    if (!digitalRead(arm_control_0)) {
      arm0.ccw(100);
    } else if (!digitalRead(arm_control_1)) {
      arm1.cw(100);
    }
    
    
  } else {
    if (!digitalRead(arm_control_0)) {
      arm0.cw(100);
    } else if (!digitalRead(arm_control_1)) {
      arm1.ccw(100);
    }
  }

}
