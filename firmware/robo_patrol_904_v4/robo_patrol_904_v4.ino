#include <ros.h>                 //иницилизация ros билиотеки
#include <geometry_msgs/Twist.h> //иницилизация библиотеки содеражажей geometry_msgs

#define M1_DRV_PIN_1 4
#define M1_DRV_PIN_2 5
#define M1_ENC_PIN 16

#define M2_DRV_PIN_1 6
#define M2_DRV_PIN_2 7
#define M2_ENC_PIN 18

#define M3_DRV_PIN_1 8
#define M3_DRV_PIN_2 9
#define M3_ENC_PIN 20

#define M4_DRV_PIN_1 10
#define M4_DRV_PIN_2 11
#define M4_ENC_PIN 22

ros::NodeHandle_<ArduinoHardware, 25, 25, 2048, 2048> nh; // инициализация параметров ноды

float base_linear = 0.0;
float base_angular = 0.0;

void cmd_vel_cb_f(const geometry_msgs::Twist &twist);                         // инициальзация функции для подписчика "/cmd_vel"
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/cmd_vel", &cmd_vel_cb_f); // инициальзация подписчика "/cmd_vel"

unsigned long led_toggle_last_time = 0.0;

void my_main();

void M1_enc_f();
void M2_enc_f();
void M3_enc_f();
void M4_enc_f();

void M1_write(int pwm);
void M2_write(int pwm);
void M3_write(int pwm);
void M4_write(int pwm);

int M1_PWM = 0;
int M2_PWM = 0;
int M3_PWM = 0;
int M4_PWM = 0;

int M1_enc = 0;
int M2_enc = 0;
int M3_enc = 0;
int M4_enc = 0;

double M1_err = 0.0;
double M2_err = 0.0;
double M3_err = 0.0;
double M4_err = 0.0;

double M1_err_prev = 0.0;
double M2_err_prev = 0.0;
double M3_err_prev = 0.0;
double M4_err_prev = 0.0;


void setup()
{
    pinMode(M1_DRV_PIN_1, OUTPUT);
    pinMode(M1_DRV_PIN_2, OUTPUT);
    pinMode(M1_ENC_PIN, INPUT_PULLDOWN);

    pinMode(M2_DRV_PIN_1, OUTPUT);
    pinMode(M2_DRV_PIN_2, OUTPUT);
    pinMode(M2_ENC_PIN, INPUT_PULLDOWN);

    pinMode(M3_DRV_PIN_1, OUTPUT);
    pinMode(M3_DRV_PIN_2, OUTPUT);
    pinMode(M3_ENC_PIN, INPUT_PULLDOWN);

    pinMode(M4_DRV_PIN_1, OUTPUT);
    pinMode(M4_DRV_PIN_2, OUTPUT);
    pinMode(M4_ENC_PIN, INPUT_PULLDOWN);

    attachInterrupt(M1_ENC_PIN, M1_enc_f, CHANGE);
    attachInterrupt(M2_ENC_PIN, M2_enc_f, CHANGE);
    attachInterrupt(M3_ENC_PIN, M3_enc_f, CHANGE);
    attachInterrupt(M4_ENC_PIN, M4_enc_f, CHANGE);

    nh.initNode();
    nh.subscribe(cmd_vel_sub);
}

void loop()
{
    if (nh.connected())
    {
        my_main();
        if (millis() - led_toggle_last_time > 1000)
        {
            toggleLED();
            led_toggle_last_time = millis();
        }
    }
    else
    {
        if (millis() - led_toggle_last_time > 100)
        {
            toggleLED();
            led_toggle_last_time = millis();
        }
    }
    nh.spinOnce();
}

void cmd_vel_cb_f(const geometry_msgs::Twist &twist)
{
    base_linear = twist.linear.x;
    base_angular = twist.angular.z;
}

void my_main()
{

    motors_write(M1_PWM, M2_PWM, M3_PWM, M4_PWM);
}



void motors_write(int m1_pwm, int m2_pwm, int m3_pwm, int m4_pwm)
{
    M1_write(m1_pwm);
    M2_write(m2_pwm);
    M3_write(m3_pwm);
    M4_write(m4_pwm);
}

void M1_write(int pwm)
{
    int pin1 = M1_DRV_PIN_1;
    int pin2 = M1_DRV_PIN_2;
    if (pwm > 0)
    {
        analogWrite(pin1, pwm);
        analogWrite(pin2, 0);
    }
    else if (pwm < 0)
    {
        analogWrite(pin2, abs(pwm));
        analogWrite(pin1, 0);
    }
    else
    {
        analogWrite(pin2, 0);
        analogWrite(pin1, 0);
    }
}

void M2_write(int pwm)
{
    int pin1 = M2_DRV_PIN_1;
    int pin2 = M2_DRV_PIN_2;
    if (pwm > 0)
    {
        analogWrite(pin1, pwm);
        analogWrite(pin2, 0);
    }
    else if (pwm < 0)
    {
        analogWrite(pin2, abs(pwm));
        analogWrite(pin1, 0);
    }
    else
    {
        analogWrite(pin2, 0);
        analogWrite(pin1, 0);
    }
}

void M3_write(int pwm)
{
    int pin1 = M3_DRV_PIN_1;
    int pin2 = M3_DRV_PIN_2;
    if (pwm > 0)
    {
        analogWrite(pin1, pwm);
        analogWrite(pin2, 0);
    }
    else if (pwm < 0)
    {
        analogWrite(pin2, abs(pwm));
        analogWrite(pin1, 0);
    }
    else
    {
        analogWrite(pin2, 0);
        analogWrite(pin1, 0);
    }
}

void M4_write(int pwm)
{
    int pin1 = M4_DRV_PIN_1;
    int pin2 = M4_DRV_PIN_2;
    if (pwm > 0)
    {
        analogWrite(pin1, pwm);
        analogWrite(pin2, 0);
    }
    else if (pwm < 0)
    {
        analogWrite(pin2, abs(pwm));
        analogWrite(pin1, 0);
    }
    else
    {
        analogWrite(pin2, 0);
        analogWrite(pin1, 0);
    }
}

void M1_enc_f()
{
    if (M1_PWM > 0)
    {
        M1_enc++;
    }
    else if (M1_PWM < 0)
    {
        M1_enc--;
    }
}

void M2_enc_f()
{
    if (M2_PWM > 0)
    {
        M2_enc++;
    }
    else if (M2_PWM < 0)
    {
        M2_enc--;
    }
}

void M3_enc_f()
{
    if (M3_PWM > 0)
    {
        M3_enc++;
    }
    else if (M3_PWM < 0)
    {
        M3_enc--;
    }
}

void M4_enc_f()
{
    if (M4_PWM > 0)
    {
        M4_enc++;
    }
    else if (M4_PWM < 0)
    {
        M4_enc--;
    }
}

