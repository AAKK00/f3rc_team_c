#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <Adafruit_BNO055.h>
#include <moter.h>
#include <motermovement.h>
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

#define width 50
#define length 50



//Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire); //ICSの名前, デフォルトアドレス, 謎
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);


//ここからは距離センサーの設定
#define ADDRESS_DEFAULT 0b0101001 // 0x29
#define ADDRESS_00 (ADDRESS_DEFAULT + 2)

// 使用する距離センサーの数
#define SENSOR_NUM  4

//使用する距離センサーのリセットのピン
#define SENSOR0 23
#define SENSOR1 19
#define SENSOR2 18
#define SENSOR3 17

const int GPIO_MASK_ARRAY[SENSOR_NUM] = {SENSOR0, SENSOR1, SENSOR2, SENSOR3};
VL53L0X gSensor[SENSOR_NUM]; // 使用するセンサークラス配列

//モータのピンの設定
#define moter0_0 13
#define moter0_1 12
#define moter0_2 14
#define moter0_3 27
#define moter1_0 26
#define moter1_1 25
#define moter1_2 33
#define moter1_3 32
#define moter2_0 15
#define moter2_1 2
#define moter2_2 0
#define moter2_3 4
#define moter3_0 16
#define moter3_1 17
#define moter3_2 5
#define moter3_3 18

//モーターのclass
//Moter.h
Moter moter0, moter1, moter2, moter3;
//Motermovement.h
Motermovement Drive;


//距離センサーから得られた座標を保存しておく場所
typedef struct dis{
  int x0, x1, y0, y1;
} Distance;

typedef struct location{
  double x, y, z;
} Location;

//9軸センサーから得られた角度を保存しておく場所
//基本的にはクオータニオンを利用して計算するのでほとんど使わない
typedef struct angles{
  double roll, yaw, pitch;
} Angles;


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

void vl53l0xget(Distance *dis) {
  for (int i = 0; i < SENSOR_NUM; i++) {
    Serial.print(gSensor[i].readRangeSingleMillimeters());
    if (gSensor[i].timeoutOccurred()) { 
      Serial.print(" TIMEOUT"); 
    }
    Serial.println();
    
  }
  dis->x0 = gSensor[0].readRangeSingleMillimeters();
  dis->x1 = gSensor[1].readRangeSingleMillimeters();
  dis->y0 = gSensor[2].readRangeSingleMillimeters();
  dis->y1 = gSensor[3].readRangeSingleMillimeters();
}


void quat_to_euler(Angles *p, double w, double x, double y, double z) {
  //double roll, pitch, yaw;
  double ysqr = y * y;

  // roll (x-axis rotation)
  double t0 = +2.0 * (w * x + y * z);
  double t1 = +1.0 - 2.0 * (x * x + ysqr);
  p->roll = atan2(t0, t1);

  // pitch (y-axis rotation)
  double t2 = +2.0 * (w * y - z * x);
  t2 = t2 > 1.0 ? 1.0 : t2;
  t2 = t2 < -1.0 ? -1.0 : t2;
  p->pitch = asin(t2);

  // yaw (z-axis rotation)
  double t3 = +2.0 * (w * z + x * y);
  double t4 = +1.0 - 2.0 * (ysqr + z * z);  
  p->yaw = atan2(t3, t4);

  /*
  // 180 / PI
  roll *= 57.2957795131;
  pitch *= 57.2957795131;
  yaw *= 57.2957795131;
  */
}

double rotated_accel[3], raw_accel[3];
double vxyz[3];

void getlocation(Location *l, double vzyz[3], double accxyz[3], double dt){
  for(int i = 0; i < 3; i++){
    vxyz[i] += accxyz[i] * dt / 1000;
  }

  l->x += dt * vxyz[0] / 1000;
  l->y += dt * vxyz[1] / 1000;  
  l->z += dt * vxyz[2] / 1000;
}


void setup() {
  // Wire(Arduino-I2C)の初期化
  Wire.begin();
  // デバッグ用シリアル通信は115200bps
  Serial.begin(115200);


  if(!bno.begin()) {
    Serial.println("Cannot start BNO055!");
  }

  bno.setExtCrystalUse(false);
  
  
  if(!vl53l0xInit()){
    Serial.println("VL53L0X initialization failed!");
  }




  moter0.set(moter0_0, moter0_1, moter0_2, moter0_3);
  moter1.set(moter1_0, moter1_1, moter1_2, moter1_3);
  moter1.set(moter2_0, moter2_1, moter2_2, moter2_3);
  moter1.set(moter3_0, moter3_1, moter3_2, moter3_3);

  Drive.set(moter0, moter1, moter2, moter3);
}


uint16_t starttime[2] = {0, 0};

Distance dis;
Angles rpy;
Location xyz;

void loop() {
  
  starttime[1] = millis();

  imu::Quaternion quat = bno.getQuat();

  //左手系から右手系への変換
  //bno055は左手系らしい、頭おかしい
  double q1 = -quat.w();
  double q2 = quat.x();
  double q3 = -quat.y();
  double q4 = quat.z();
  

  quat_to_euler(&rpy, q1, q2, q3, q4);


  /*
  //回転行列の表現行列
  double a[3][3] = {{q1*q1-q2*q2-q3*q3+q4*q4, 2*(q1*q2+q3*q4), 2*(q1*q3-q2*q4)},
                    {2*(q1*q2-q3*q4), -q1*q1+q2*q2-q3*q3+q4*q4, 2*(q2*q3+q1*q4)},
                    {2*(q1*q3+q2*q4), 2*(q2*q3-q1*q4), -q1*q1-q2*q2+q3*q3+q4*q4}};
                    
  */

  double 
  sr = sin(-rpy.roll),
  cr = cos(-rpy.roll),

  sp = sin(-rpy.pitch),
  cp = cos(-rpy.pitch),

  sy = sin(-rpy.yaw),
  cy = cos(-rpy.yaw);

  //しょうがないから普通の回転行列にしました
  double c[3][3] = 
  {{cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr},
  {sy*cp, sr*sp*sy + cr*cy, sy*sp*cr - cy*sr},
  {-sp, cp*sr, cp*cr}};


  // 加速度センサ値の取得と表示
  imu::Vector<3> accelermetor = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  /*
  Serial.print("Ac_xyz:");
  Serial.print(accelermetor.x());
  Serial.print(", ");
  Serial.print(accelermetor.y());
  Serial.print(", ");
  Serial.print(accelermetor.z());
  */

  //ここでも左手系から右手系に変換しています死ね
  //rotated_locationは右手系です
  //double rotated_accel[3];

  raw_accel[0] = accelermetor.y();
  raw_accel[1] = accelermetor.x();
  raw_accel[2] = accelermetor.z();
  
  for(int i = 0; i < 3; i++) {
    for(int j = 0; j < 3; j++) {
      raw_accel[i] += c[i][j] * raw_accel[j];
    }
  }

  
  double g = 9.63;
  rotated_accel[2] -= g;
  
  for(int i = 0; i < 3; i++){

    if(rotated_accel[i] <= 0.1){
      rotated_accel[i] = -0.09;
    }
  }


  Serial.print(rpy.roll);
  Serial.print(",");
  Serial.print(rpy.pitch);
  Serial.print(",");
  Serial.println(rpy.yaw);



  vl53l0xget(&dis);

  double rotated_dis[3];
  for(int i = 0; i < 4; i++){
    if (dis.x0 == 8190){
      dis.x0 = dis.x1;
    }
    if(dis.y0 == 8190){
      dis.y0 = dis.y1;
    }
    int raw_dis[3] = {dis.x0, dis.y0, 0};
    for(int i = 0; i < 3; i++) {
      for(int j = 0; j < 3; j++) {
      rotated_dis[i] += c[i][j] * raw_dis[j];
      }
    }
  }

  Serial.print(" x0:");
  Serial.print(rotated_dis[0]);
  Serial.print(" y0:");
  Serial.println(rotated_dis[1]);





  int dt = starttime[1] - starttime[0];

  getlocation(&xyz, vxyz, rotated_accel, dt);
  Serial.print("x:");
  Serial.print(xyz.x, 3);

  Serial.print(" y:");
  Serial.print(xyz.y, 3);

  Serial.print(" z:");
  Serial.println(xyz.z,3);

  Serial.println("---------------------------------------------------------------------------------");

  delay(100);

  starttime[0] = starttime[1];
}
