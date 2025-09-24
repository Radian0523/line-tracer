#include "ESC2nd.h"

Sensor sensor(DEVICE_ADDR1);
uint16_t raw[8];
Motor motorR('r');
Motor motorL('l');

// キャリブレーション値
uint16_t whiteValue = 3000; // 白の時の値（要調整）
uint16_t blackValue = 200;  // 黒の時の値（要調整）
float threshold = 0.3;      // ライン検出閾値(0.0-1.0)

// PIDパラメータ
float Kp = 70;   // 比例ゲイン（基本の曲がり量）
float Ki = 0.02;  // 積分ゲイン（定常偏差除去）
float Kd = 0.2;   // 微分ゲイン（振動抑制）

// PID状態変数
float previousError = 0;
float integral = 0;
unsigned long lastTime = 0;

// モーター制御パラメータ
int baseSpeed = 30;     // 基本速度
int maxSpeed = 50;      // 最大速度
int minSpeed = 10;      // 最小速度
int maxCorrection = 25; // 最大補正量

void setup(){
  Serial.begin(9600);
  delay(1000);
  lastTime = millis();
}

void loop(){
  sensor.read(raw);
  
  // 経過時間計算
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0; // 秒単位
  lastTime = currentTime;
  
  // ライン位置計算
  float linePosition = calculateLinePosition() + 0.7;
  
  if(linePosition < 0){
    // ラインを見失った場合の処理
    handleLostLine();
  }
  else{
    // PID制御で曲がる量を計算
    float turnAmount = calculatePID(linePosition, deltaTime);
    
    // モーター制御
    controlMotors(turnAmount);
    
    // デバッグ表示
    Serial.print("Pos:");
    Serial.print(linePosition, 1);
    Serial.print(" Error:");
    Serial.print(linePosition - 3.5, 1);
    Serial.print(" PID:");
    Serial.print(turnAmount, 1);
    Serial.print(" L:");
    Serial.print(baseSpeed + turnAmount);
    Serial.print(" R:");
    Serial.println(baseSpeed - turnAmount);
  }
  checkSerialCommands();
  
  delay(20); // 制御周期50Hz
}

// 精密なライン位置計算（0.0〜7.0）
float calculateLinePosition(){
  float sum = 0;
  float weightSum = 0;
  int activeSensors = 0;
  
  for(int i = 0; i < 8; i++){
    // 黒さを計算（0.0〜1.0）
    float darkness = calculateDarkness(raw[i]);
    
    if(darkness > threshold){
      // センサー位置の重み付け（中央付近を重視してもよい）
      // float sensorWeight = 1.0; // 均等重み
      float sensorWeight = 1.0 - abs(i - 3.5) / 4.0; // 中央重視
      
      sum += i * darkness * sensorWeight;
      weightSum += darkness * sensorWeight;
      activeSensors++;
    }
  }
  
  if(weightSum < 0.001 || activeSensors == 0){
    return -1; // ライン未検出
  }
  
  return sum / weightSum;
}

// 黒さの計算（正規化）
float calculateDarkness(uint16_t sensorValue){
  float darkness = (float)(whiteValue - sensorValue) / (whiteValue - blackValue);
  return constrain(darkness, 0.0, 1.0);
}

// PID制御計算
float calculatePID(float currentPosition, float deltaTime){
  float setpoint = 3.5; // 目標位置（中央）
  float error = currentPosition - setpoint;
  
  // 比例項
  float proportional = Kp * error;
  
  // 積分項（ウィンドウアップ防止）
  integral += error * deltaTime;
  integral = constrain(integral, -2.0, 2.0); // 積分制限
  float integralTerm = Ki * integral;
  
  // 微分項（デルタタイム考慮）
  float derivative = (error - previousError) / deltaTime;
  float derivativeTerm = Kd * derivative;
  previousError = error;
  
  // PID合算
  float output = proportional + integralTerm + derivativeTerm;
  
  // 出力制限
  output = constrain(output, -maxCorrection, maxCorrection);
  
  return output;
}

// モーター制御
void controlMotors(float turnAmount){
  int leftSpeed = baseSpeed + turnAmount;
  int rightSpeed = baseSpeed - turnAmount;
  
  // 速度制限
  leftSpeed = constrain(leftSpeed, minSpeed, maxSpeed);
  rightSpeed = constrain(rightSpeed, minSpeed, maxSpeed);
  
  motorL.drive(leftSpeed);
  motorR.drive(rightSpeed);
}

// ラインロスト処理
void handleLostLine(){
  // 最後の誤差に基づいて旋回
  if(previousError > 0){
    // 最後に右に逸れていた → 左を探す
    motorL.drive(-20);
    motorR.drive(20);
  }
  else{
    // 最後に左に逸れていた → 右を探す
    motorL.drive(20);
    motorR.drive(-20);
  }
  Serial.println("Line lost - searching");
}

// PIDパラメータ調整用（シリアル通信で変更可能）
void checkSerialCommands(){
  if(Serial.available()){
    char command = Serial.read();
    switch(command){
      case 'p': Kp += 0.1; break;
      case 'P': Kp -= 0.1; break;
      case 'i': Ki += 0.01; break;
      case 'I': Ki -= 0.01; break;
      case 'd': Kd += 0.05; break;
      case 'D': Kd -= 0.05; break;
      case 's': baseSpeed += 5; break;
      case 'S': baseSpeed -= 5; break;
      case ' ':
        // 現在のパラメータ表示
        Serial.print("Kp:"); Serial.print(Kp);
        Serial.print(" Ki:"); Serial.print(Ki);
        Serial.print(" Kd:"); Serial.print(Kd);
        Serial.print(" Speed:"); Serial.println(baseSpeed);
        break;
    }
    
    // パラメータ制限
    Kp = constrain(Kp, 0.1, 3.0);
    Ki = constrain(Ki, 0.0, 0.1);
    Kd = constrain(Kd, 0.0, 1.0);
    baseSpeed = constrain(baseSpeed, 10, 50);
  }
}