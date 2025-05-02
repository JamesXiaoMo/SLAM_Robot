#include <MotorWheel.h>

class SpeedConvertor
{
private:
    MotorWheel* _wheelBack;
    MotorWheel* _wheelRight;
    MotorWheel* _wheelLeft;
public:
    float robot_R = 0.125;
    float wheel_R = 0.05;
    float wheel_L = 0.1;
    float sin60 = 0.866025;
    SpeedConvertor(MotorWheel* wheelBack, MotorWheel* wheelRight, MotorWheel* wheelLeft);
    void speedConvertor(float Vx, float Vy, float Vangle);    // Vx [m/s], Vy [m/s], Vangle [rad/s]
};
