#include "ESC2nd.h"

Sensor sensor(DEVICE_ADDR1);
uint16_t raw[8];
Motor motorR('r');
Motor motorL('l');

// 3種類のタイルパターン（右回転、左回転、まっすぐ）の基準値を定義
// 各パターンは8個のセンサ値の期待パターン（1:線なし, 0:線あり）
const uint8_t PATTERN_STRAIGHT[8] = {1, 1, 1, 0, 0, 1, 1, 1};  // 中央のセンサが線を検知
const uint8_t PATTERN_RIGHT[8]    = {1, 1, 1, 1, 0, 0, 1, 1};  // 右側のセンサが線を検知
const uint8_t PATTERN_LEFT[8]     = {1, 1, 0, 0, 1, 1, 1, 1};  // 左側のセンサが線を検知

// センサの閾値（環境に応じて調整）
const uint16_t SENSOR_THRESHOLD = 1600;

void setup(){
  Serial.begin(9600);
  delay(1000);
}

// センサの生値をバイナリ値（0/1）に変換
void readSensorBinary(uint8_t binary[8]) {
  sensor.read(raw);
  raw[0] += 1000;
  raw[7] += 1000;
  for (int i = 0; i < 8; i++) {
    binary[i] = (raw[i] > SENSOR_THRESHOLD) ? 1 : 0;
  }
}

// 2つのバイナリパターンの距離（不一致数）を計算
int calculateDistance(uint8_t pattern1[8], const uint8_t pattern2[8]) {
  int distance = 0;
  for (int i = 0; i < 8; i++) {
    if (pattern1[i] != pattern2[i]) {
      distance++;
    }
  }
  return distance;
}

void loop(){
  uint8_t current[8];
  readSensorBinary(current);

  for (int i = 0; i < 8; i++){
  Serial.print(current[i]);
Serial.print(" ");
  }
  Serial.println();
  
  // 各パターンとの距離を計算
  int distStraight = calculateDistance(current, PATTERN_STRAIGHT);
  int distRight = calculateDistance(current, PATTERN_RIGHT);
  int distLeft = calculateDistance(current, PATTERN_LEFT);
  
  // 最小距離を見つけて対応する動作を選択
  int minDist = distStraight;
  int action = 0; // 0:直進, 1:右折, 2:左折
  
  if (distRight < minDist) {
    minDist = distRight;
    action = 1;
  }
  if (distLeft < minDist) {
    minDist = distLeft;
    action = 2;
  }
  
  // 動作の実行
  switch (action) {
    case 0: // 直進
      motorR.drive(80);
      motorL.drive(80);
      break;
    case 1: // 右折
      motorR.drive(20);
      motorL.drive(100);
      break;
    case 2: // 左折
      motorR.drive(100);
      motorL.drive(20);
      break;
  }
  
  delay(2);// 制御周期の調整
}