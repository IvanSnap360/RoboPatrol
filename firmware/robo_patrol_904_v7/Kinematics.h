#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__

class Kinematics
{
private:
    struct angular
    {
        float x, y, z;
    };
    struct linear
    {
        float x, y, z;
    };

    struct velocity
    {
        linear linear;
        angular angular;
    };


    float _wheel_base_width;
    float _wheel_base_long;

    float _wheel_deametr;
    float _wheel_radius;

    

public:
    Kinematics();

    velocity velocity;
    struct actuators
    {
        float motor1;
        float motor2;
        float motor3;
        float motor4;
    };

    struct IKoutput
    {
        float x,y,z;
    };

    void setWheelBaseWidth(float l);
    void setWheelBaseLong(float l);
    void setWheelDeametr(float d);
    actuators computeActuatorsVelocities();
    IKoutput computeIK();
};

#endif // __KINEMATICS_H__