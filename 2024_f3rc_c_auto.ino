#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <Adafruit_BNO055.h>
#include "ooruidriver.h"
#include "motormovement.h"
#include <math.h>
#include <stdio.h>
//#include <Ticker.h>

void getlocation();
//Ticker getlocationticker(getlocation, 50);

/*
#include <Ticker.h>
tickerの書き方のメモですが括弧内で関数、何秒でそれを実行するか、何回繰り返すかを宣言してください
以下が基本の形です
Ticker hoge(hogehoge, 0) 関数hogehogeを0usで実行する
void VL53L0X_Get();
Ticker VL53L0Xticker(VL53L0X_Get, 0); 
*/

#define width 420
#define length 375
#define field_width 2100
#define field_length 2400



//Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire); //ICSの名前, デフォルトアドレス, 謎
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);


//ここからは距離センサーの設定
#define ADDRESS_DEFAULT 0b0101001 // 0x29
#define ADDRESS_00 (ADDRESS_DEFAULT + 2)

// 使用する距離センサーの数
#define SENSOR_NUM  4

//使用する距離センサーのリセットのピン
#define SENSOR0 21
#define SENSOR1 19
#define SENSOR2 18
#define SENSOR3 22

const int GPIO_MASK_ARRAY[SENSOR_NUM] = {SENSOR0, SENSOR1, SENSOR2, SENSOR3};
VL53L0X gSensor[SENSOR_NUM]; // 使用するセンサークラス配列

//モータのピンの設定
#define motor0_0 13
#define motor0_1 12

#define motor1_0 26
#define motor1_1 25

#define motor2_0 26
#define motor2_1 25

#define motor3_0 33
#define motor3_1 32

#define pick 4
#define arm_push 16
#define eject 17


//モーターのclass
//Motor.h
ooruidriver motor0, motor1, motor2, motor3;
//Motermovement.h
motormovement Drive;


//距離センサーから得られた座標を保存しておく場所
struct TofDistance{
  int x0, x1, y0, y1;
};

//加速度センサーから得られた座標を保存しておく場所
struct Location{
  double x, y, z;
};

//9軸センサーから得られた角度を保存しておく場所
//基本的にはクオータニオンを利用して計算するのでほとんど使わない
struct Angles{
  double roll, yaw, pitch;
};


bool vl53l0xInit() {
  // まず全てのGPIOをLOW
  for (int i = 0; i < SENSOR_NUM; i++)
  {
    pinMode(GPIO_MASK_ARRAY[i], OUTPUT);
    digitalWrite(GPIO_MASK_ARRAY[i], LOW);
  }

  for (int i = 0; i < SENSOR_NUM; i++) {
    // センサを初期化
    pinMode(GPIO_MASK_ARRAY[i], INPUT);
    if (gSensor[i].init() == true)
    {
      gSensor[i].setTimeout(1000);
      gSensor[i].startContinuous(10);
      int address = ADDRESS_00 + (i * 2);
      gSensor[i].setAddress(address);
    }
    else
    {
      Serial.print("Sensor ");
      Serial.print(i);
      Serial.println(" error");
      return(false);
    }
  }
  return(true);
}

//TofDistanceに格納する
void vl53l0xget(TofDistance *t) {
  for (int i = 0; i < SENSOR_NUM; i++) {
    Serial.print(gSensor[i].readRangeSingleMillimeters());
    if (gSensor[i].timeoutOccurred()) { 
      Serial.print(" TIMEOUT"); 
    }
    Serial.println();
    
  }
  t->x0 = gSensor[0].readRangeSingleMillimeters();
  t->x1 = gSensor[1].readRangeSingleMillimeters();
  t->y0 = gSensor[2].readRangeSingleMillimeters();
  t->y1 = gSensor[3].readRangeSingleMillimeters();
}


void quat_to_euler(Angles *a, double w, double x, double y, double z) {
  //double roll, pitch, yaw;
  double ysqr = y * y;

  // roll (x-axis rotation)
  double t0 = +2.0 * (w * x + y * z);
  double t1 = +1.0 - 2.0 * (x * x + ysqr);
  a->roll = atan2(t0, t1);

  // pitch (y-axis rotation)
  double t2 = +2.0 * (w * y - z * x);
  t2 = t2 > 1.0 ? 1.0 : t2;
  t2 = t2 < -1.0 ? -1.0 : t2;
  a->pitch = asin(t2);

  // yaw (z-axis rotation)
  double t3 = +2.0 * (w * z + x * y);
  double t4 = +1.0 - 2.0 * (ysqr + z * z);  
  a->yaw = atan2(t3, t4);

  /*
  // 180 / PI
  roll *= 57.2957795131;
  pitch *= 57.2957795131;
  yaw *= 57.2957795131;
  */
}

/*
double vxyz[3];
//加速度センサーから得られた値を2回積分する vxyz[3]を関数内で定義すると、毎回初期化されてしまい値を保存できない


void caliculate_location(Location *l, double vzyz[3], double accxyz[3], u_int8_t dt){
  for(int i = 0; i < 3; i++){
    vxyz[i] += accxyz[i] * dt / 1000;
  }

  l->x += dt * vxyz[0] / 1000;
  l->y += dt * vxyz[1] / 1000;
  l->z += dt * vxyz[2] / 1000;
}
*/





//dtを測定するための実行時間を計測する配列
u_int16_t pretime;


TofDistance dis;
Angles rpy, a0;
Location xyz;

double g = 9.806;
double calib_acc;

/*
void calib(Angles *a0, u_int8_t num, double calib_acc){
  delay(1000);
  double a0xyz[3];
  for(int i = 0; i < num; i++) {
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    a0xyz[0] += accel.x();
    a0xyz[1] += accel.y();
    a0xyz[2] += accel.z();
  }
  for(int i = 0; i < 3; i++){
    a0xyz[i] /= num;
  }
  calib_acc = g / pow(pow(a0xyz[0], 2) + pow(a0xyz[1], 2) + pow(a0xyz[2], 2), 1 / 3);
  a0->roll  = atan(a0xyz[2] / a0xyz[1]);
  a0->pitch = atan(a0xyz[2] / a0xyz[0]);
  a0->yaw   = atan(a0xyz[1] / a0xyz[2]);
}
*/



void setup() {
  // Wire(Arduino-I2C)の初期化
  Wire.begin();
  // デバッグ用シリアル通信は115200bps
  Serial.begin(115200);


  if(!bno.begin()) {
    Serial.println("Cannot start BNO055!");
  }

  bno.setExtCrystalUse(true);

  //calib(&a0, 100, calib_acc);
  
  
  if(!vl53l0xInit()){
    Serial.println("VL53L0X initialization failed!");
  }



  motor0.set(motor0_0, motor0_1);
  motor1.set(motor1_0, motor1_1);
  motor2.set(motor2_0, motor2_1);
  motor3.set(motor3_0, motor3_1);

  Drive.set(motor0, motor1, motor2, motor3);

  pinMode(arm_push, OUTPUT);
  pinMode(pick, OUTPUT);
  pinMode(eject, OUTPUT);
}


void getlocation() {

  imu::Quaternion quat = bno.getQuat();


  double q1 = quat.w();
  double q2 = quat.x();
  double q3 = quat.y();
  double q4 = quat.z();
  

  quat_to_euler(&rpy, q1, q2, q3, q4);


  /*
  //回転行列の表現行列
  double a[3][3] = {{q1*q1-q2*q2-q3*q3+q4*q4, 2*(q1*q2+q3*q4), 2*(q1*q3-q2*q4)},
                    {2*(q1*q2-q3*q4), -q1*q1+q2*q2-q3*q3+q4*q4, 2*(q2*q3+q1*q4)},
                    {2*(q1*q3+q2*q4), 2*(q2*q3-q1*q4), -q1*q1-q2*q2+q3*q3+q4*q4}};
                    
  */

 /*
  double 
  sr = sin(-rpy.roll - a0.roll),
  cr = cos(-rpy.roll - a0.roll),

  sp = sin(-rpy.pitch - a0.pitch),
  cp = cos(-rpy.pitch - a0.pitch),

  sy = sin(-rpy.yaw - a0.yaw),
  cy = cos(-rpy.yaw - a0.yaw);

  //しょうがないから普通の回転行列にしました
  double r[3][3] = 
  {{cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr},
  {sy*cp, sr*sp*sy + cr*cy, sy*sp*cr - cy*sr},
  {-sp, cp*sr, cp*cr}};
  */





  // 加速度センサ値の取得と表示
  //imu::Vector<3> accelermetor = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  /*
  Serial.print("Ac_xyz:");
  Serial.print(accelermetor.x());
  Serial.print(", ");
  Serial.print(accelermetor.y());
  Serial.print(", ");
  Serial.print(accelermetor.z());
  */

  /*
  //rotated_locationは右手系です
  double rotated_accel[3], raw_accel[3];

  raw_accel[0] = calib_acc * accelermetor.x();
  raw_accel[1] = calib_acc * accelermetor.y();
  raw_accel[2] = calib_acc * accelermetor.z();
  
  for(int i = 0; i < 3; i++) {
    for(int j = 0; j < 3; j++) {
      raw_accel[i] += r[i][j] * raw_accel[j];
    }
  }

  

  rotated_accel[2] -= g;
  
  for(int i = 0; i < 3; i++){

    if(rotated_accel[i] <= 0.05){
      rotated_accel[i] = 0.0;
    }
  }
  */


  Serial.print(rpy.roll);
  Serial.print(",");
  Serial.print(rpy.pitch);
  Serial.print(",");
  Serial.print(rpy.yaw);
  Serial.println();



  vl53l0xget(&dis);


  /*
  ______x1_____
  |            |
  |            |
y1|            |y0
  |            |
  |____________|
        x0

        ↑x
        |
        |
  ←------
  y

  */


  int raw_dis[3];
  double rotated_dis[3];
  if(dis.x0 == 8190 || dis.x0 == 0){
    raw_dis[0] = field_length - dis.x1 - length / 2;
  }
  else{
    raw_dis[0] = dis.x0 + length / 2;
  }

  if(dis.y0 == 8190 || dis.y0 == 0){
    raw_dis[1] = field_width - dis.x1 - width / 2;
  }
  else{
    raw_dis[1] = dis.y0 + width / 2;
  }

  if (45.0 < rpy.yaw && rpy.yaw < 135.0) {
    if(dis.x0 == 8190 || dis.x0 == 0){
      raw_dis[0] = field_width - dis.x1 - length / 2;
    }
    else{
      raw_dis[0] = dis.x0 + length / 2;
    }

    if(dis.y0 == 8190 || dis.y0 == 0){
      raw_dis[1] = field_length - dis.x1 - width / 2;
    }
    else{
      raw_dis[1] = dis.y0 + width / 2;
    }
  }

  /*
  if (dis.x0 == 8190 && dis.x1 == 8190) {
    Drive.stop();
  } else if (dis.y0 == 8190 && dis.y1 == 8190) {
    Drive.stop();
  }
  */

  raw_dis[2] = 0;

  /*
  for(int i = 0; i < 3; i++) {
    for(int j = 0; j < 3; j++) {
    rotated_dis[i] += r[i][j] * raw_dis[j];
    }
  }
  */

  dis.x0 = raw_dis[0];
  dis.y0 = raw_dis[1]; 


  

  Serial.print(" x from tof:");
  Serial.print(dis.x0);
  Serial.print(" y from tof:");
  Serial.print(dis.y0);
  Serial.println();




  u_int8_t dt = millis() - pretime;

  //caliculate_location(&xyz, vxyz, rotated_accel, dt);

  /*
  double ratio = 0.5;
  xyz.x = ratio * rotated_dis[0] + (1 - ratio) * xyz.x;
  xyz.y = ratio * rotated_dis[0] + (1 - ratio) * xyz.y;
  */

  Serial.print("x:");
  Serial.print(xyz.x, 7);

  Serial.print(" y:");
  Serial.print(xyz.y, 7);

  Serial.print(" z:");
  Serial.print(xyz.z, 7);
  Serial.println();

  delay(100);

  pretime = millis();
}


int dest[5][2] = {{250, 1200}, {1200, 650}, {650, 600}, {600, 1200}, {1200, 1450}};

float P_d = 0.3, P_r = 3.0;

double pwm_max = 200;

float diff_x, diff_y, diff_rotate;

void loop(){
  digitalWrite(pick, LOW);
  digitalWrite(arm_push, LOW);
  digitalWrite(eject, LOW);
  
for (int i = 0; i < 5; i++) {
    Serial.print("destination : ");
    Serial.print(dest[i][0]);
    Serial.print(", ");
    Serial.println(dest[i][1]);
    
    if (i == 0){
      while (abs(diff_y) > 20.0) {
        getlocation();
        diff_x = (dis.x0 - dest[i][0]) * P_d;
        diff_y = (dis.y0 - dest[i][1]) * P_d;
        if (diff_y > pwm_max) {
          for (diff_y = 0; diff_y < pwm_max; diff_y++) {
            Drive.forward(diff_y);
            delay(10);
          }
          break;
          diff_y = pwm_max;
        } else if (diff_y <= pwm_max) {
          Drive.forward(diff_y);
        }
         
        if (abs(diff_x) > 50.0) {
          if (diff_x >= 0) {
            if (diff_x >= pwm_max){
              diff_x = pwm_max;
            }
          Drive.left(diff_x);
          } else if (diff_x < 0) {
            if (abs(diff_x) >= pwm_max) {
              diff_x = pwm_max;
            }
            Drive.right(abs(diff_x));
          }
        }
      }
      Serial.println("i == 0 : done");
    } 


    while (diff_rotate >= 2.0) {
      diff_rotate = 90 - rpy.yaw * P_r;
      if (diff_rotate > pwm_max) {
          for (diff_rotate = 0; diff_rotate < pwm_max; diff_rotate++) {
            Drive.ccw(diff_rotate);
            delay(10);
          }
          break;
          diff_rotate = pwm_max;
      } else if (diff_rotate <= pwm_max) {
          Drive.ccw(diff_rotate);
      }
    }

    if (i == 1) {
      while (abs(diff_y) > 20.0) {
        getlocation();
        diff_x = (dis.x0 - dest[i][0]) * P_d;
        diff_y = (dis.y0 - dest[i][1]) * P_d;
        if (diff_y > pwm_max) {
          for (diff_y = 0; diff_y < pwm_max; diff_y++) {
            Drive.forward(diff_y);
            delay(10);
          }
          break;
          diff_y = pwm_max;
        } else if (diff_y <= pwm_max) {
          Drive.forward(diff_y);
        }
         
        if (abs(diff_x) > 50.0) {
          if (diff_x >= 0) {
            if (diff_x >= pwm_max){
              diff_x = pwm_max;
            }
          Drive.left(diff_x);
          } else if (diff_x < 0) {
            if (abs(diff_x) >= pwm_max) {
              diff_x = pwm_max;
            }
            Drive.right(abs(diff_x));
          }
        } 
      }
      Serial.println("i == 1 : done");
    }

    while (diff_rotate >= 2.0) {
      diff_rotate = 0 - rpy.yaw * P_r;
      if (diff_rotate > pwm_max) {
          for (diff_rotate = 0; diff_rotate < pwm_max; diff_rotate++) {
            Drive.cw(diff_rotate);
            delay(10);
          }
          break;
          diff_rotate = pwm_max;
        } else if (diff_rotate <= pwm_max) {
          Drive.cw(diff_rotate);
      }
    }

    if (i == 2) {
      while (abs(diff_y) > 20.0) {
        getlocation();
        diff_x = (dis.x0 - dest[i][0]) * P_d;
        diff_y = (dis.y0 - dest[i][1]) * P_d;
        if (diff_y > pwm_max) {
          for (diff_y = 0; diff_y < pwm_max; diff_y++) {
            Drive.forward(diff_y);
            delay(10);
          }
          break;
          diff_y = pwm_max;
        } else if (diff_y <= pwm_max) {
          Drive.forward(diff_y);
        }
         
        if (abs(diff_x) > 50.0) {
          if (diff_x >= 0) {
            if (diff_x >= pwm_max){
              diff_x = pwm_max;
            }
          Drive.left(diff_x);
          } else if (diff_x < 0) {
            if (abs(diff_x) >= pwm_max) {
              diff_x = pwm_max;
            }
            Drive.right(abs(diff_x));
          }
        }
      }
      Serial.println("i == 2 : done");
    }

    digitalWrite(arm_push, HIGH);
    delay(100);
    digitalWrite(pick, HIGH);
    delay(2000);
    digitalWrite(arm_push, LOW);
    delay(1000);

    if (i == 3) {
      while (abs(diff_y) > 20.0) {
        getlocation();
        diff_x = (dis.x0 - dest[i][0]) * P_d;
        diff_y = (dis.y0 - dest[i][1]) * P_d;
        if (diff_y > pwm_max) {
          for (diff_y = 0; diff_y < pwm_max; diff_y++) {
            Drive.back(diff_y);
            delay(10);
          }
          break;
          diff_y = pwm_max;
        } else if (diff_y <= pwm_max) {
          Drive.back(diff_y);
        }
         
        if (abs(diff_x) > 50.0) {
          if (diff_x >= 0) {
            if (diff_x >= pwm_max){
              diff_x = pwm_max;
            }
          Drive.left(diff_x);
          } else if (diff_x < 0) {
            if (abs(diff_x) >= pwm_max) {
              diff_x = pwm_max;
            }
            Drive.right(abs(diff_x));
          }
        }
      }
      Serial.println("i == 3 : done");
    }

    while (diff_rotate >= 2.0) {
      diff_rotate = 90 - rpy.yaw * P_r;
      if (diff_rotate > pwm_max) {
          for (diff_rotate = 0; diff_rotate < pwm_max; diff_rotate++) {
            Drive.cw(diff_rotate);
            delay(10);
          }
          break;
          diff_rotate = pwm_max;
        } else if (diff_rotate <= pwm_max) {
          Drive.cw(diff_rotate);
        }
      }
    
    if (i == 4) {
      while (abs(diff_y) > 20.0) {
        getlocation();
        diff_x = (dis.x0 - dest[i][0]) * P_d;
        diff_y = (dis.y0 - dest[i][1]) * P_d;
        if (diff_y > pwm_max) {
          for (diff_y = 0; diff_y < pwm_max; diff_y++) {
            Drive.forward(diff_y);
            delay(10);
          }
          break;
          diff_y = pwm_max;
        } else if (diff_y <= pwm_max) {
          Drive.forward(diff_y);
        }
         
        if (abs(diff_x) > 50.0) {
          if (diff_x >= 0) {
            if (diff_x >= pwm_max){
              diff_x = pwm_max;
            }
          Drive.left(diff_x);
          } else if (diff_x < 0) {
            if (abs(diff_x) >= pwm_max) {
              diff_x = pwm_max;
            }
            Drive.right(abs(diff_x));
          }
        }
      }
      Serial.println("i == 4 : done");
    }

    digitalWrite(eject, HIGH);
    delay(900);
    digitalWrite(eject, LOW);
  }
  while (1) {
    Serial.println("BON APPETIT");
        delay(1000);
  }
}