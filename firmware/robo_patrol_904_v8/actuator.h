#ifndef __ACTUATOR_H__
#define __ACTUATOR_H__
#include <Arduino.h>
class ACTUATOR
{
private:
    int _m_pin1;
    int _m_pin2;

    int _pwm_max;
    int _pwm_min;

    float _ang_vel;

    int _min_pwm_val;

    int _pwm_cntrl;

    int _rev_tiks;

    int _dir;

    float _wd;

    unsigned long spin_last_time;

    int getMotorValue(float value)
    {
        return map(value * 100, -(2 * M_PI * 100), (2 * M_PI * 100), -255, 255);
    }

public:
    ACTUATOR();
    void setPins(int m_pin1, int m_pin2, int enc_pin);
    void setTiksPerRevolution(unsigned long tiks);
    void setWheelDeametr(float d);
    void writeAngularVelocity(float vel);
    void setMinPWMvalue(int min_pwm_signal);
    void setPWMRanges(int min_pwm, int max_pwm);
    void spin(unsigned long frequency);
    int getDirection();

    int _enc_pin;
    long rel_tiks;
    unsigned long abs_tiks;

    float impulse2meters(float x)
    {
        return (x / _rev_tiks) * M_PI * _wd;
    }

    float impulse2rad(float x)
    {
        return (x / _rev_tiks) * 2.0 * M_PI;
    }
};

#endif // __ACTUATOR_H__