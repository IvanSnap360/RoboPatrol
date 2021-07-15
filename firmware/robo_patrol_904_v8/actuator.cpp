#include "actuator.h"

ACTUATOR::ACTUATOR() {}

void ACTUATOR::setPins(int pin1, int pin2, int e_pin)
{
    _m_pin1 = pin1;
    _m_pin2 = pin2;
    _enc_pin = e_pin;

    pinMode(_m_pin1, OUTPUT);
    pinMode(_m_pin2, OUTPUT);
    pinMode(_enc_pin, INPUT_PULLDOWN);

    digitalWrite(_m_pin1, LOW);
    digitalWrite(_m_pin2, LOW);
}

void ACTUATOR::setPWMRanges(int min_val, int max_val)
{
    _pwm_max = max_val;
    _pwm_min = min_val;
}

void ACTUATOR::setMinPWMvalue(int val)
{
    _min_pwm_val = val;
}

void ACTUATOR::setWheelDeametr(float d)
{
    _wd = d;
}

void ACTUATOR::writeAngularVelocity(float vel)
{
    _ang_vel = vel;
    _pwm_cntrl = getMotorValue(_ang_vel);
}

void ACTUATOR::setTiksPerRevolution(unsigned long tiks)
{
    _rev_tiks = tiks;
}

int ACTUATOR::getDirection()
{
    return _dir;
}

void ACTUATOR::spin(unsigned long period)
{
    if (millis() - spin_last_time > period)
    {
        spin_last_time = millis();
    }

    int pwm = constrain(_pwm_cntrl, _pwm_min, _pwm_max);

    // if (pwm < 0 && pwm >= -_min_pwm_val)
    //     pwm = -_min_pwm_val;
    // if (pwm > 0 && pwm <= _min_pwm_val)
    //     pwm = _min_pwm_val;

    if (_pwm_cntrl > 0)
    {
        analogWrite(_m_pin1, abs(pwm));
        analogWrite(_m_pin2, 0);
        _dir = 1;
    }
    else if (_pwm_cntrl < 0)
    {
        analogWrite(_m_pin1, 0);
        analogWrite(_m_pin2, abs(pwm));
        _dir = -1;
    }
    else
    {
        analogWrite(_m_pin1, 0);
        analogWrite(_m_pin2, 0);
        _dir = 0;
    }
}