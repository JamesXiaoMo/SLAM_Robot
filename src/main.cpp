#include <Arduino.h>
#include <MotorWheel.h>
#include <Omni3WD.h>
#include <PID_Beta6.h>
#include <PinChangeInt.h>
#include <PinChangeIntConfig.h> 

const float robot_R = 0.125;
const float wheel_R = 0.05;
const float sin60 = 0.866025;
const float pulsesPerRevolution = 12.0;

long lastP1 = 0, lastP2 = 0, lastP3 = 0;
float robotX = 0, robotY = 0, robotTheta = 0;


irqISR(irq1,isr1);
MotorWheel wheel1(9,8,6,7,&irq1);
irqISR(irq2,isr2);
MotorWheel wheel2(10,11,14,15,&irq2);
irqISR(irq3,isr3);
MotorWheel wheel3(3,2,4,5,&irq3);


Omni3WD Omni(&wheel1,&wheel2,&wheel3);


void speedConvertor(float Vx, float Vy, float Vangle){
  Serial.println("--- Speed Convertor ---");
  Serial.print("Vx     = "); Serial.println(Vx, 4);
  Serial.print("Vy     = "); Serial.println(Vy, 4);
  Serial.print("Vangle = "); Serial.println(Vangle, 4);
  Serial.println("------------------------");
  float V1 = Vx + robot_R * Vangle;
  float V2 = -Vx/2 + Vy * sin60 + robot_R * Vangle;
  float V3 = -Vx/2 + -Vy * sin60 + robot_R * Vangle;
  Serial.print("V1     = "); Serial.println(V1, 4);
  Serial.print("V2     = "); Serial.println(V2, 4);
  Serial.print("V3     = "); Serial.println(V3, 4);
  Serial.println("------------------------");
  wheel1.setSpeedMMPS(V1 * 1000);
  wheel2.setSpeedMMPS(V2 * 1000);
  wheel3.setSpeedMMPS(V3 * 1000);
}

void setup() {
  Serial.begin(115200);
  TCCR1B=TCCR1B&0xf8|0x01; // Timer1.Pin9,Pin10 PWM 31250Hz
  TCCR2B=TCCR2B&0xf8|0x01; // Timer2 .Pin3,Pin11 PWM 31250Hz
  wheel1.PIDEnable(0.26,0.02,0,10);
  wheel2.PIDEnable(0.26,0.02,0,10);
  wheel3.PIDEnable(0.26,0.02,0,10);
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.startsWith("V ")) {
      int firstSpace = cmd.indexOf(' ');
      int secondSpace = cmd.indexOf(' ', firstSpace + 1);
      int thirdSpace = cmd.indexOf(' ', secondSpace + 1);

      float v_x = cmd.substring(firstSpace + 1, secondSpace).toFloat();
      float v_y = cmd.substring(secondSpace + 1, thirdSpace).toFloat();
      float omega = cmd.substring(thirdSpace + 1).toFloat();
      speedConvertor(v_x, v_y, omega);
    }
  }

  wheel1.PIDRegulate();
  wheel2.PIDRegulate();
  wheel3.PIDRegulate();

}