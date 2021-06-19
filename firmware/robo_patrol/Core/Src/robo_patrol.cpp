#include <main.h>
#include <arduino_functions.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <stm32f4xx_hal.h>

#define MIN_PWM 0
#define MAX_PWM 65535
#define WHEEL_DEAMETR 1.0
#define BASE_WIDTH 1.0

ros::NodeHandle_<STM32Hardware, 25, 25, 2048, 2048> nh;


void driver_cb(const geometry_msgs::Twist &msg);
void mpu_publisher();


ros::Subscriber<geometry_msgs::Twist> driver_sub("/driver_cmd", &driver_cb);



float linear = 0;
float angular = 0;

void setup()
{
    nh.initNode();
    nh.subscribe(driver_sub);
    
}
void loop()
{          

    if (nh.connected())
    {
        TIM4->CCR1 = 65535;
        TIM4->CCR3 = 0;
    }
    else
    {
        TIM4->CCR3 = 65535;
        TIM4->CCR1 = 0;
    }
    nh.spinOnce();
    
}

void driver_cb(const geometry_msgs::Twist &msg)
{
    linear = msg.linear.x;
    angular = msg.angular.z;
}

int getMotorValue(float value)
{
    return map(value * 1000, -1000, 1000, MIN_PWM, MAX_PWM);
}

void mpu_publisher()
{
    
}
