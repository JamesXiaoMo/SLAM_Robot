#include <SpeedConvertor.h>

SpeedConvertor::SpeedConvertor(MotorWheel* wheel1, MotorWheel* wheel2, MotorWheel* wheel3) {
    // 构造函数实现
}

void SpeedConvertor::speedConvertor(float Vx, float Vy, float Vangle){
    float V1 = Vx + robot_R * Vangle;
    float V2 = -Vx/2 + Vy * sin60 + robot_R * Vangle;
    float V3 = -Vx/2 + Vy * sin60 + robot_R * Vangle;
    Serial.print(" V1: ");
    Serial.println(String(V1));
    Serial.print(" V2: ");
    Serial.println(String(V2));
    Serial.print(" V3: ");
    Serial.println(String(V3));
    SpeedConvertor::_wheelBack->setGearedSpeedRPM((V1 * 60)/(PI * 0.1));
    SpeedConvertor::_wheelRight->setGearedSpeedRPM((V2 * 60)/(PI * 0.1));
    SpeedConvertor::_wheelLeft->setGearedSpeedRPM((V3 * 60)/(PI * 0.1));
}