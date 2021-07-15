#ifndef __ACTUATOR_H__
#define __ACTUATOR_H__
#include <Arduino.h>
class ACTUATOR
{
private:
    int _pwm_cntrl;
    int _pwm_max;
    int _pwm_min;

    int _min_pwm_val;

    int _max_rpm;
    int _cur_rpm;

    int _m_pin1;
    int _m_pin2;

    int _rev_tiks;
    unsigned long _prev_tiks;

    unsigned long spin_last_time;
    unsigned long prev_update_time_;

    int _dir = 0;
    
    void calcRPM()
    {
        unsigned long current_time = millis();
        unsigned long dt = current_time - prev_update_time_;
        double dtm = (double)dt / 60000;
        double delta_ticks = _encoderTiks - _prev_tiks;
        prev_update_time_ = current_time;
        _prev_tiks = _encoderTiks;
        _cur_rpm = (delta_ticks / _rev_tiks) / dtm;
    }
    

public:
    ACTUATOR();
    void setPins(int pin1,int pin2, int e_pin);
    void setMaxRPM(int max_rpm);
    void setTiksPerRevolution(unsigned long tiks);
    void setPWMRanges(int min_val,int max_val);
    void setMinPWMvalue(int val);
    void writePWM(int pwm);
    void writeRPM(int rpm);
    int getCurrentRPM();
    int getDirection();
    void spin(unsigned long period);

    volatile long _encoderTiks;
    volatile long _encoderTiks2;
    int _enc_pin;

};




#endif // __ACTUATOR_H__