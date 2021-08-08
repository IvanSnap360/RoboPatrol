#ifndef __ACTUATOR_H__
#define __ACTUATOR_H__

class ACTUATOR
{
private:
    void enc();
    long _encoder_counter = 0.0;
    float _angular_speed = 0.0;
    float _regular_speed = 0.0;

    int _m_pin1 = 0;
    int _m_pin2 = 0;
    int _enc_pin = 0;

    unsigned long last_time = 0.0;

public:
    ACTUATOR();
    void init(int m_pin1, int m_pin2, int enc_pin, bool revers);
    void setAngularSpeed(float speed);
    float kP = 0.0;
    float kI = 0.0;
    float kD = 0.0;
    void spin(unsigned long time_ms = 1000);
};

#endif // __ACTUATOR_H__