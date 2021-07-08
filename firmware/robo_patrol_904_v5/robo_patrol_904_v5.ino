#include <ros.h>
#include "actuator.h"
#include <geometry_msgs/Twist.h>

#define LEFT 0
#define RIGHT 1
#define FORWARD 0
#define BACKWARD 1

ACTUATOR actuator[2][2];

#define M1 actuator[LEFT][FORWARD]
#define M2 actuator[LEFT][BACKWARD]
#define M3 actuator[RIGHT][FORWARD]
#define M4 actuator[RIGHT][BACKWARD]

#define WHEEL_DEAMETR 0.135
#define WHEEL_RADIUS WHEEL_DEAMETR / 2
#define Lx 0.175 / 2
#define BASE_WIDTH 0.250
const float rad2rpm_constant = 9.549297;

#define MIN_PWM -255
#define MAX_PWM  255

void M1_enc_f()
{
    M1.encoder_tiks++;
}

void M2_enc_f()
{
    M2.encoder_tiks++;
}

void M3_enc_f()
{
    M3.encoder_tiks++;
}

void M4_enc_f()
{
    M4.encoder_tiks++;
}

ros::NodeHandle_<ArduinoHardware, 25, 25, 2048, 2048> nh;

void cmd_vel_cb_f(const geometry_msgs::Twist &vels);
ros::Subscriber<geometry_msgs::Twist> cmd_vel_subscriber("/cmd_vel", &cmd_vel_cb_f);

unsigned long blink_last_time = 0;
unsigned long odom_calc_last_time = 0;

float base_linear = 0.0;
float base_angular = 0.0;
void setup()
{
    M1.init(4, 5, 16);
    M1.setRPM_max(95);
    M1.setPWM_ranges(-255, 255);
    M1.setTiksPerRevolution(1225);
    attachInterrupt(M1._enc_pin, M1_enc_f, CHANGE);

    M2.init(8, 9, 17);
    M2.setRPM_max(95);
    M2.setPWM_ranges(-255, 255);
    M2.setTiksPerRevolution(1225);
    attachInterrupt(M2._enc_pin, M2_enc_f, CHANGE);

    M3.init(6, 7, 18);
    M3.setRPM_max(95);
    M3.setPWM_ranges(-255, 255);
    M3.setTiksPerRevolution(1225);
    attachInterrupt(M3._enc_pin, M3_enc_f, CHANGE);

    M4.init(10, 11, 19);
    M4.setRPM_max(95);
    M4.setPWM_ranges(-255, 255);
    M4.setTiksPerRevolution(1225);
    attachInterrupt(M4._enc_pin, M4_enc_f, CHANGE);

    nh.initNode();
    nh.subscribe(cmd_vel_subscriber);
}

void loop()
{
    if (nh.connected())
    {
        if (millis() - blink_last_time > 1000)
        {
            toggleLED();
            blink_last_time = millis();
        }
        if (millis() - odom_calc_last_time > 10)
        {

            odom_calc_last_time = millis();
        }

        float V = -base_linear;        //линейная скорость
        float W = base_angular;        //угловая скорость
        float r = WHEEL_DEAMETR / 2.0; //радиус колеса
        float L = BASE_WIDTH;          //база робота

        // вычисление требуемой скорости вращения колес (рад/с)
        double left = (r * ((1.0 / r) * V - (L / r) * W));
        double right = (r * ((1.0 / r) * V + (L / r) * W));

        left = map(left * 1000, -1000, 1000, MIN_PWM, MAX_PWM);
        right = map(right * 1000, -1000, 1000, MIN_PWM, MAX_PWM);

        M1.writeTargetPWM(left);
        M2.writeTargetPWM(left);
        M3.writeTargetPWM(-right);
        M4.writeTargetPWM(right);
    }
    else
    {
        M1.writeTargetPWM(0);
        M2.writeTargetPWM(0);
        M3.writeTargetPWM(0);
        M4.writeTargetPWM(0);
        if (millis() - blink_last_time > 100)
        {
            toggleLED();
            blink_last_time = millis();
        }
    }

    M1.spin(100);
    M2.spin(100);
    M3.spin(100);
    M4.spin(100);
    nh.spinOnce();
}

void cmd_vel_cb_f(const geometry_msgs::Twist &vels)
{
    base_linear = vels.linear.x;
    base_angular = vels.angular.z;
}

