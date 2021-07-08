#include "actuator.h"
#include <Arduino.h>
#include "actuator.h"

ACTUATOR::ACTUATOR()
{
}

void ACTUATOR::init(int M_pin1, int M_pin2, int E_pin)
{
    this->_m_pin1 = M_pin1;
    this->_m_pin2 = M_pin2;
    this->_e_pin = E_pin;
}
void ACTUATOR::setRevolutionTiks(int counts_per_rev_)
{
    _counts_per_rev_ = counts_per_rev_;
}

void ACTUATOR::setMaxRPM(int rpm)
{
    max_rpm_ = rpm;
}

void ACTUATOR::enc_cb()
{
    if (_pwm > 0)
    {
        encoder_ticks++;
    }
    if (_pwm < 0)
    {
        encoder_ticks--;
    }
}

void ACTUATOR::spin(unsigned long frequency_hz)
{
    if (millis() - _last_spin_time > (1000 / frequency_hz))
    {
        unsigned long current_time = millis();
        unsigned long dt = current_time - prev_update_time_;
        double dtm = (double)dt / 60000;
        double delta_ticks = encoder_ticks - prev_encoder_ticks_;
        _cur_rpm = (delta_ticks / _counts_per_rev_) / dtm;
        prev_update_time_ = current_time;
        prev_encoder_ticks_ = encoder_ticks;

        _last_spin_time = millis();
    }

    double res = ACTUATOR::compute(_req_rpm,_cur_rpm);
    _pwm = (((double) res / (double) max_rpm_) * 255);

    if (_pwm > 0)
    {
        analogWrite(_m_pin1, 0);
        analogWrite(_m_pin2, abs(_pwm));
    }
    else if (_pwm < 0)
    {
        analogWrite(_m_pin1, 0);
        analogWrite(_m_pin2, abs(_pwm));
    }
    else
    {
        analogWrite(_m_pin1, 0);
        analogWrite(_m_pin2, 0);
    }
}

double ACTUATOR::compute(float setpoint, float measured_value)
{
  double error;
  double pid;

  //setpoint is constrained between min and max to prevent pid from having too much error
  error = setpoint - measured_value;
  integral_ += error;
  derivative_ = error - prev_error_;

  if(setpoint == 0 && error == 0)
  {
    integral_ = 0;
  }

  pid = (K_P * error) + (K_I * integral_) + (K_D * derivative_);
  prev_error_ = error;

  return constrain(pid, min_val_, max_val_);
}