#ifndef __ACTUATOR_H__
#define __ACTUATOR_H__
#include <Arduino.h>
// #include "GyverPID.h"
class ACTUATOR
{
private:
    int _m_pin1;
    int _m_pin2;

    int _min_pwm;
    int _max_pwm;
    int _pwm;

    int _max_rpm;

    float _targetRPM;
    float _currentRPM;

    int _rev_tiks;
    long long _prev_tiks;

    unsigned long last_spin_time = 0;
    unsigned long prev_update_time_ = 0;

    int _dir;
    // GyverPID pid;

    void calcRPM()
    {
        unsigned long current_time = millis();
        unsigned long dt = current_time - prev_update_time_;
        double dtm = (double)dt / 60000;
        double delta_ticks = encoder_tiks - _prev_tiks;
        prev_update_time_ = current_time;
        _prev_tiks = encoder_tiks;
        _currentRPM = (delta_ticks / _rev_tiks) / dtm;
    }

    float runMiddleArifmOptim(float newVal)
    {
        static int t = 0;
        static int vals[10];
        static int average = 0;
        if (++t >= 10)
            t = 0;          // перемотка t
        average -= vals[t]; // вычитаем старое
        average += newVal;  // прибавляем новое
        vals[t] = newVal;   // запоминаем в массив
        return ((float)average / 10);
    }
    int rpmToPWM(int rpm)
    {
        //remap scale of target RPM vs MAX_RPM to PWM
        return (((double)rpm / (double)_max_rpm) * 255);
    }

    float pid_res = 0.0;

public:
    ACTUATOR();
    void init(int m_pin1, int m_pin2, int enc_pin);

    int _enc_pin;
    long long encoder_tiks = 0;

    // float K_P = 0.0;
    // float K_I = 0.0;
    // float K_D = 0.0;

    void setRPM_max(int max_rpm);
    void setPWM_ranges(int min_pwm, int max_pwm);
    void setTiksPerRevolution(int tiks);

    void writeTargetRPM(float target_rpm);

    void writeTargetPWM(int pwm);

    void spin(unsigned long period);
};

#endif // __ACTUATOR_H__