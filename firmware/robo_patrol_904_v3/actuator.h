#ifndef __ACTUATOR_H__
#define __ACTUATOR_H__
#include "Motor.h"
#include "PID.h"

class ACTUATOR 
{
private:
    int _pwm;
    int _m_pin1;
    int _m_pin2;
    

    float _req_rpm;
    float _cur_rpm;
    
    unsigned long _last_spin_time = 0;

    long prev_encoder_ticks_;
    unsigned long prev_update_time_;
    int _counts_per_rev_ = 0;
    long encoder_ticks;
    

    float min_val_;
    float max_val_;
    double integral_;
    double derivative_;
    double prev_error_;
    double max_rpm_;

public:
    ACTUATOR();
    void enc_cb();
    int _e_pin;
    void init(int M_pin1,int M_pin2, int E_pin);
    void setTargetRPMVelocity(float req_rmp);
    void setRevolutionTiks(int counts_per_rev_);
    void setMaxRPM(int rpm);
    double compute(float setpoint, float measured_value);

    float K_P = 0.0;
    float K_I = 0.0;
    float K_D = 0.0;


    void spin(unsigned long frequency_hz);
    
};

#endif // __ACTUATOR_H__