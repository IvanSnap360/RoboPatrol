#include <ros.h>
#include <geometry_msgs/Twist.h>
#include "Motor.h"
#include "Kinematics.h"
#include "PID.h"
bool debug = true;
#define DEBUG_PORT Serial3
ros::NodeHandle_<ArduinoHardware, 25, 25, 2048, 2048> nh;

Motor motor1;
Motor motor2;
Motor motor3;
Motor motor4;
int Motor::counts_per_rev_ = 1225;

Kinematics kinematics(95, 0.135, 0.25, 8);

#define K_P 0
#define K_I 0
#define K_D 0
#define MAX_RPM 95

PID motor1_pid(-255, 255, K_P, K_I, K_D);
PID motor2_pid(-255, 255, K_P, K_I, K_D);
PID motor3_pid(-255, 255, K_P, K_I, K_D);
PID motor4_pid(-255, 255, K_P, K_I, K_D);

float base_linear = 0.0;
float base_angular = 0.0;
unsigned long base_move_last_time = 0.0;

void moveBase();
void cmd_vel_cb(const geometry_msgs::Twist &vels);
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/cmd_vel", &cmd_vel_cb);

long m1_enc_counter = 0;
long m2_enc_counter = 0;
long m3_enc_counter = 0;
long m4_enc_counter = 0;

void m1_enc()
{
    m1_enc_counter++;
}
void m2_enc()
{
    m2_enc_counter++;
}
void m3_enc()
{
    m3_enc_counter++;
}
void m4_enc()
{
    m4_enc_counter++;
}

void setup()
{
    if (debug)
        DEBUG_PORT.begin(9600);
    motor1.init(4, 5);
    motor2.init(6, 7);  
    motor3.init(8, 9);  
    motor4.init(10, 11);    

    attachInterrupt(16,m1_enc,CHANGE);
    attachInterrupt(18,m2_enc,CHANGE);
    attachInterrupt(20,m3_enc,CHANGE);
    attachInterrupt(22,m4_enc,CHANGE);

    // nh.getHardware()->setBaud(1000000);
    nh.initNode();
    nh.subscribe(cmd_vel_sub);
}

void moveBase()
{
    Kinematics::output req_rpm;

    req_rpm = kinematics.getRPM(base_linear, 0.0, base_angular);

    motor1.spin(motor1_pid.compute(constrain(req_rpm.motor1, -MAX_RPM, MAX_RPM), motor1.rpm));
    motor3.spin(motor3_pid.compute(constrain(req_rpm.motor3, -MAX_RPM, MAX_RPM), motor3.rpm));
    motor2.spin(motor2_pid.compute(constrain(req_rpm.motor2, -MAX_RPM, MAX_RPM), motor2.rpm));
    motor4.spin(motor4_pid.compute(constrain(req_rpm.motor4, -MAX_RPM, MAX_RPM), motor4.rpm));

    DEBUG_PORT.print(motor1_pid.compute(constrain(req_rpm.motor1, -MAX_RPM, MAX_RPM), motor1.rpm));
    DEBUG_PORT.print("  ");
    DEBUG_PORT.print(req_rpm.motor2);
    DEBUG_PORT.print("  ");
    DEBUG_PORT.print(req_rpm.motor3);
    DEBUG_PORT.print("  ");
    DEBUG_PORT.print(req_rpm.motor4);
    DEBUG_PORT.println("  ");
}

void loop()
{
    if (nh.connected())
    {
        if (millis() - base_move_last_time > 500)
        {
            moveBase();
            motor1.updateSpeed(m1_enc_counter);
            motor3.updateSpeed(m3_enc_counter);
            motor2.updateSpeed(m2_enc_counter);
            motor4.updateSpeed(m4_enc_counter);
            digitalWrite(14, !digitalRead(14));
            base_move_last_time = millis();
        }
    }
    else
    {
        digitalWrite(14, !digitalRead(14));
        delay(100);
    }

    nh.spinOnce();
}


void cmd_vel_cb(const geometry_msgs::Twist &vels)
{
    base_linear = vels.linear.x;
    base_angular = vels.angular.z;
}