#include <Arduino.h>
#include <MotorWheel.h>
#include <Omni3WD.h>
#include <PID_Beta6.h>
#include <PinChangeInt.h>
#include <PinChangeIntConfig.h> 
// #include <SpeedConvertor.h>

float robot_R = 0.125;
float wheel_R = 0.05;
float wheel_L = 0.1;
float sin60 = 0.866025;

irqISR(irq1,isr1);
MotorWheel wheel1(9,8,6,7,&irq1);
irqISR(irq2,isr2);
MotorWheel wheel2(10,11,14,15,&irq2);
irqISR(irq3,isr3);
MotorWheel wheel3(3,2,4,5,&irq3);


Omni3WD Omni(&wheel1,&wheel2,&wheel3);

void speedConvertor(float Vx, float Vy, float Vangle){
  float V1 = Vx + robot_R * Vangle;
  float V2 = -Vx/2 + Vy * sin60 + robot_R * Vangle;
  float V3 = -Vx/2 + -Vy * sin60 + robot_R * Vangle;
  Serial.print(" V1: ");
  Serial.println(String(V1));
  Serial.print(" V2: ");
  Serial.println(String(V2));
  Serial.print(" V3: ");
  Serial.println(String(V3));
  wheel1.setGearedSpeedRPM((V1 * 60)/(PI * 0.1));
  wheel2.setGearedSpeedRPM((V2 * 60)/(PI * 0.1));
  wheel3.setGearedSpeedRPM((V3 * 60)/(PI * 0.1));
}

void setup() {
  Serial.begin(9600);
  TCCR1B=TCCR1B&0xf8|0x01; // Timer1.Pin9,Pin10 PWM 31250Hz
  TCCR2B=TCCR2B&0xf8|0x01; // Timer2 .Pin3,Pin11 PWM 31250Hz
  wheel1.PIDEnable(0.26,0.02,0,10);
  wheel2.PIDEnable(0.26,0.02,0,10);
  wheel3.PIDEnable(0.26,0.02,0,10);
  //speedConvertor(0,0,-1);
}

void loop() {
  wheel1.PIDRegulate();
  wheel2.PIDRegulate();
  wheel3.PIDRegulate();
  Serial.print("Wheel1 Speed: ");
  Serial.print(wheel1.getSpeedMMPS());
  Serial.print("  Wheel2 Speed: ");
  Serial.print(wheel2.getSpeedMMPS());
  Serial.print("  Wheel3 Speed: ");
  Serial.println(wheel3.getSpeedMMPS());
  if (Serial.available()) {
    char command = Serial.read();
    if (command == 's') {
      speedConvertor(0, 0, 0);
    }else if (command == 'w') {
      speedConvertor(0, 0.3, 0);
    }
  }
}