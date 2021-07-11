#include "Kinematics.h"

Kinematics::Kinematics() {}

void Kinematics::setWheelBaseLong(float l)
{
    _wheel_base_long = l;
}

void Kinematics::setWheelBaseWidth(float l)
{
    _wheel_base_width = l;
}


Kinematics::IKoutput Kinematics::computeIK()
{

}

Kinematics::actuators Kinematics::computeActuatorsVelocities()
{
    
}

