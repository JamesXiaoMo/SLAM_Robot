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


void updateOdometry() {
  long p1 = wheel1.getCurrPulse();
  long p2 = wheel2.getCurrPulse();
  long p3 = wheel3.getCurrPulse();

  long dp1 = p1 - lastP1;
  long dp2 = p2 - lastP2;
  long dp3 = p3 - lastP3;

  lastP1 = p1; lastP2 = p2; lastP3 = p3;

  float distance1 = (dp1 / pulsesPerRevolution) * 2 * PI * wheel_R;
  float distance2 = (dp2 / pulsesPerRevolution) * 2 * PI * wheel_R;
  float distance3 = (dp3 / pulsesPerRevolution) * 2 * PI * wheel_R;

  float dx = (-2.0/3.0)*distance1 + (1.0/3.0)*(distance2 + distance3);
  float dy = (sqrt(3)/3.0)*(distance2 - distance3);
  float dtheta = (1.0/robot_R)*(distance1 + distance2 + distance3)/3.0;

  robotX += dx;
  robotY += dy;
  robotTheta += dtheta;
}

void speedConvertor(float Vx, float Vy, float Vangle){
  Serial.println("--- Speed Convertor ---");
  Serial.print("Vx     = "); Serial.println(Vx, 4);
  Serial.print("Vy     = "); Serial.println(Vy, 4);
  Serial.print("Vangle = "); Serial.println(Vangle, 4);
  Serial.println("------------------------");
  float V1 = Vx + robot_R * Vangle;
  float V2 = -Vx/2 + Vy * sin60 + robot_R * Vangle;
  float V3 = -Vx/2 + -Vy * sin60 + robot_R * Vangle;
  Serial.print("Vx     = "); Serial.println(V1, 4);
  Serial.print("Vy     = "); Serial.println(V2, 4);
  Serial.print("Vangle = "); Serial.println(V3, 4);
  Serial.println("------------------------");
  wheel1.setGearedSpeedRPM((V1 * 60)/(PI * 0.1));
  wheel2.setGearedSpeedRPM((V2 * 60)/(PI * 0.1));
  wheel3.setGearedSpeedRPM((V3 * 60)/(PI * 0.1));
}

void setup() {
  Serial.begin(115200);
  TCCR1B=TCCR1B&0xf8|0x01; // Timer1.Pin9,Pin10 PWM 31250Hz
  TCCR2B=TCCR2B&0xf8|0x01; // Timer2 .Pin3,Pin11 PWM 31250Hz
  wheel1.PIDEnable(0.26,0.02,0,10);
  wheel2.PIDEnable(0.26,0.02,0,10);
  wheel3.PIDEnable(0.26,0.02,0,10);
  //speedConvertor(0,0,-1);
}

// void loop() {
//   updateOdometry();
//   wheel1.PIDRegulate();
//   wheel2.PIDRegulate();
//   wheel3.PIDRegulate();
  // Serial.print("Wheel1 Speed: ");
  // Serial.print(wheel1.getSpeedMMPS());
  // Serial.print("  Wheel2 Speed: ");
  // Serial.print(wheel2.getSpeedMMPS());
  // Serial.print("  Wheel3 Speed: ");
  // Serial.println(wheel3.getSpeedMMPS());
  // Serial.print("Dx: ");
  // Serial.print(robotX);
  // Serial.print("  Dy: ");
  // Serial.print(robotY);
  // Serial.print("  Dw: ");
  // Serial.println(robotTheta);
  // if (Serial.available()) {
  //   char command = Serial.read();
  //   if (command == 's') {
  //     speedConvertor(0, 0, 0);
  //   }else if (command == 'w') {
  //     speedConvertor(0, 0, 1);
  //   }
  // }
// }

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

      Serial.println("Parsed via substring:");
      Serial.println("Vx: " + String(v_x) + " Vy: " + String(v_y) + " Omega: " + String(omega));
      speedConvertor(v_x, v_y, omega);
    }
  }

  updateOdometry();
  Serial.print("O ");
  Serial.print(robotX / 1000.0, 4); Serial.print(" ");
  Serial.print(robotY / 1000.0, 4); Serial.print(" ");
  Serial.println(robotTheta, 4);

  wheel1.PIDRegulate();
  wheel2.PIDRegulate();
  wheel3.PIDRegulate();

  delay(500); // 50Hz
}