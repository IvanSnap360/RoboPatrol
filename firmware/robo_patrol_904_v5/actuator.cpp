#include "actuator.h"

ACTUATOR::ACTUATOR()
{
}

void ACTUATOR::init(int m_pin1, int m_pin2, int enc_pin)
{
    _m_pin1 = m_pin1;
    _m_pin2 = m_pin2;
    _enc_pin = enc_pin;

    pinMode(_m_pin1, OUTPUT);
    pinMode(_m_pin2, OUTPUT);

    digitalWrite(_m_pin1, LOW);
    digitalWrite(_m_pin2, LOW);

    // pid.setDirection(NORMAL);
}

void ACTUATOR::setRPM_max(int max_rpm)
{
    _max_rpm = max_rpm;
}
void ACTUATOR::setPWM_ranges(int min_pwm, int max_pwm)
{
    _min_pwm = min_pwm;
    _max_pwm = max_pwm;
    // pid.setLimits(_min_pwm, _max_pwm);
}

void ACTUATOR::writeTargetRPM(float target_rpm)
{
    _targetRPM = target_rpm;
    // pid.setpoint = map(_targetRPM, -_max_rpm, _max_rpm, _min_pwm, _max_pwm);
}

void ACTUATOR::setTiksPerRevolution(int tiks)
{
    _rev_tiks = tiks;
}

void ACTUATOR::writeTargetPWM(int pwm)
{
    _pwm = pwm;
}

void ACTUATOR::spin(unsigned long period)
{
    if (millis() - last_spin_time > period)
    {
        last_spin_time = millis();
    }
    if (_pwm < 0)
    {
        analogWrite(_m_pin1, abs(_pwm));
        analogWrite(_m_pin2, 0);
    }
    else if (_pwm > 0)
    {
        analogWrite(_m_pin2, abs(_pwm));
        analogWrite(_m_pin1, 0);
    }
    else
    {
        analogWrite(_m_pin2, 0);
        analogWrite(_m_pin1, 0);
    }
}

// calcRPM();

// _currentRPM = runMiddleArifmOptim(_currentRPM);
// if (_targetRPM < 0)
// {
//     _currentRPM = -_currentRPM;
// }
// else if (_targetRPM > 0)
// {
//     _currentRPM = abs(_currentRPM);
// }
// _currentRPM = map(_currentRPM, -_max_rpm, _max_rpm, _min_pwm, _max_pwm);
// pid.input = _currentRPM;
// pid.Kp = K_P;
// pid.Ki = K_I;
// pid.Kd = K_D;
// pid_res = pid.getResultNow();
// pid_res = rpmToPWM(_targetRPM);