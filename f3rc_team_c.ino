#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <Adafruit_BNO055.h>
#include <moter.h>
#include <motermovement.h>
#include <math.h>
#include <Ticker.h>

void getlocation();
Ticker getlocationticker;

/*
#include <Ticker.h>
tickerの書き方のメモですが括弧内で関数、何秒でそれを実行するか、何回繰り返すかを宣言してください
以下が基本の形です
Ticker hoge(hogehoge, 0) 関数hogehogeを0usで実行する
void VL53L0X_Get();
Ticker VL53L0Xticker(VL53L0X_Get, 0); 
*/

#define width 500
#define length 500
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
typedef struct TofDistance{
  int x0, x1, y0, y1;
};

//加速度センサーから得られた座標を保存しておく場所
typedef struct Location{
  double x, y, z;
};

//9軸センサーから得られた角度を保存しておく場所
//基本的にはクオータニオンを利用して計算するのでほとんど使わない
typedef struct Angles{
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


double vxyz[3];
//加速度センサーから得られた値を2回積分する vxyz[3]を関数内で定義すると、毎回初期化されてしまい値を保存できない

void caliculate_location(Location *l, double vzyz[3], double accxyz[3], u_int8_t dt){
  for(int i = 0; i < 3; i++){
    vxyz[i] += accxyz[i] * dt / 1000;
  }
  //確か単位がmなのでmmに直して、機体の大きさを考慮して足します
  l->x += dt * vxyz[0] / 1000 * 100 + width / 2;
  l->y += dt * vxyz[1] / 1000 * 100 + length / 2;
  l->z += dt * vxyz[2] / 1000 * 100;
}




//dtを測定するための実行時間を計測する配列
u_int16_t pretime;


TofDistance dis;
Angles rpy, a0;
Location xyz;

double g = 9.806;
double calib_acc;


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

  double rotated_dis[3];
  int raw_dis[3];
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

  raw_dis[2] = 0;


  for(int i = 0; i < 3; i++) {
    for(int j = 0; j < 3; j++) {
    rotated_dis[i] += r[i][j] * raw_dis[j];
    }
  }


  

  Serial.print(" x from tof:");
  Serial.print(rotated_dis[0]);
  Serial.print(" y from tof:");
  Serial.print(rotated_dis[1]);
  Serial.println();




  u_int8_t dt = millis() - pretime;

  caliculate_location(&xyz, vxyz, rotated_accel, dt);

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

  Serial.println("---------------------------------------------------------------------------------");

  delay(100);

  pretime = millis();
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

  calib(&a0, 100, calib_acc);
  
  
  if(!vl53l0xInit()){
    Serial.println("VL53L0X initialization failed!");
  }

  getlocationticker.attach(50, getlocation);



  moter0.set(moter0_0, moter0_1, moter0_2, moter0_3);
  moter1.set(moter1_0, moter1_1, moter1_2, moter1_3);
  moter1.set(moter2_0, moter2_1, moter2_2, moter2_3);
  moter1.set(moter3_0, moter3_1, moter3_2, moter3_3);

  Drive.set(moter0, moter1, moter2, moter3);
}


void loop(){
  
}
