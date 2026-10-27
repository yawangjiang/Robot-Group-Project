/*
 * Arduino 寻光小车
 * Yuanxi HAN
 *
 * 根据课程期末报告 3.1 Light Seeking Program（正文第 20-23 页）整理。
 * 保留原引脚、参数和控制逻辑；补充注释、Arduino.h 与函数声明。
 * 传感器接线须满足：光越强，ADC 读数越小。
 */
#include <Arduino.h>

int enA = 9;
int enB = 10;
int in1 = 6;
int in2 = 7;
int in3 = 4;
int in4 = 5;

int sensR = A0;
int sensL = A1;

const int M = 14;  // 右轮 PWM 加性补偿，用于减小直行偏移。
const int L = -3;  // 传感器补偿；原变量名为 L，实际加在右侧读数上。

int speedBase = 105;
int speedTurn = 35;
int thresh = 13;
int darkThreshold = 900;

void drive(int l, int r);

void setup() {
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  int valR = analogRead(sensR);
  int valL = analogRead(sensL);

  // 原始读数均严格大于 900 才停车；仅一侧暗不会触发停车。
  if (valR > darkThreshold && valL > darkThreshold) {
    drive(0, 0);
    Serial.print("worring R:");
    Serial.print(valR);
    Serial.print(" | L:");
    Serial.println(valL);
    return;
  }

  int diff = (valR + L) - valL;

  Serial.print("R:");
  Serial.print(valR);
  Serial.print(" | L:");
  Serial.print(valL);
  Serial.print(" | diff:");
  Serial.println(diff);

  int spdL, spdR;

  if (abs(diff) < thresh) {
    // 差值落在死区内，按直行参数输出。
    spdL = speedBase;
    spdR = speedBase + M;
  } else if (diff > 0) {
    // 左侧更亮，降低左轮 PWM，向左侧转弯。
    spdL = speedTurn;
    spdR = speedBase + M;
  } else {
    // 右侧更亮，降低右轮 PWM，向右侧转弯。
    spdL = speedBase;
    spdR = speedTurn + M;
  }

  spdL = constrain(spdL, 0, 255);
  spdR = constrain(spdR, 0, 255);
  drive(spdL, spdR);
}

void drive(int l, int r) {
  analogWrite(enA, l);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  analogWrite(enB, r);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}
