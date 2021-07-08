#include "actuator.h"
#include <Arduino.h>
ACTUATOR::ACTUATOR()
{
}
void ACTUATOR::init(int m_pin1, int m_pin2, int enc_pin, bool revers)
{
    this->_enc_pin = enc_pin;
    if (revers)
    {
        this->_m_pin1 = m_pin1;
        this->_m_pin2 = m_pin2;
    }
    else
    {
        this->_m_pin1 = m_pin2;
        this->_m_pin2 = m_pin1;
    }
    pinMode(this->_m_pin1, OUTPUT);
    pinMode(this->_m_pin2, OUTPUT);
}

void ACTUATOR::setAngularSpeed(float speed)
{
    this->_angular_speed = speed;
}

void ACTUATOR::spin(unsigned long time_ms)
{
    if (millis() - last_time > time_ms)
    {
        if (this->_angular_speed == 0)
        {
            analogWrite(this->_m_pin1, 0);
            analogWrite(this->_m_pin2, 0);
        }
        if (this->_angular_speed > 0)
        {
            analogWrite(this->_m_pin1, map(this->_regular_speed * 1000, -1000, 1000, 0, 255));
            analogWrite(this->_m_pin2, 0);
        }
        if (_angular_speed < 0)
        {
            analogWrite(this->_m_pin2, map(this->_regular_speed * 1000, -1000, 1000, 0, 255));
            analogWrite(this->_m_pin1, 0);
        }


        last_time = millis();
    }
}