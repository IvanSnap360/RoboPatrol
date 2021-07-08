#include <ros.h>
#include <geometry_msgs/Twist.h>
#include "actuator.h"
#include "Kinematics.h"

#define LEFT 0
#define RIGHT 1
#define FORWARD 0
#define BACKWARD 1

ACTUATOR actuator[2][2];
Kinematics kin;

#define M1 actuator[LEFT][FORWARD]
#define M2 actuator[LEFT][BACKWARD]
#define M3 actuator[RIGHT][FORWARD]
#define M4 actuator[RIGHT][BACKWARD]

ros::NodeHandle_<ArduinoHardware, 25, 25, 2048, 2048> nh;

void my_main(void);
void cmd_vel_cb_f(const geometry_msgs::Twist &twist);
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/cmd_vel", &cmd_vel_cb_f);

float base_linear = 0.0;
float base_angular = 0.0;

unsigned long led_toggle_last_time = 0.0;
void setup()
{
    nh.initNode();
    nh.subscribe(cmd_vel_sub);

    M1.init(4,5,16);
    M1.setMaxRPM(95);
    M1.setRevolutionTiks(1225);
    

    M2.init(6,7,18);
    M2.setMaxRPM(95);
    M2.setRevolutionTiks(1225);

    M3.init(8,9,20);
    M3.setMaxRPM(95);
    M3.setRevolutionTiks(1225);

    M4.init(10,11,22);
    M4.setMaxRPM(95);
    M4.setRevolutionTiks(1225);
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
}

void cmd_vel_cb_f(const geometry_msgs::Twist &twist)
{
    base_linear = twist.linear.x;
    base_angular = twist.angular.z;
}

void my_main()
{
    M1.spin(10);
    M2.spin(10);
    M3.spin(10);
    M4.spin(10);
}