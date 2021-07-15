#include <ros.h>
#include <ros/time.h>
#include "actuator.h"
#include "config.h"
#include <geometry_msgs/Twist.h>

ros::NodeHandle_<ROS_HARDWARE,
                 ROS_MAX_SUBSCRIBERS,
                 ROS_MAX_PUBLISHERS,
                 ROS_MAX_INPUT_BUFFER_SIZE,
                 ROS_MAX_OUTPUT_BUFFER_SIZE>
    nh;

void cmd_vel_cb_f(const geometry_msgs::Twist &vels);
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub(CMD_VELOCITY_SUBSCRIBER_TOPIC_NAME, &cmd_vel_cb_f);

geometry_msgs::Twist actual_vels;
ros::Publisher vels_pub(VELOCITY_PUBLISHER_TOPIC_NAME, &actual_vels);

Kinematics kin(ROBOT_BASE_TYPE,
               MOTOR_MAX_RPM,
               ROBOT_WHEEL_DEAMETR,
               ROBOT_BASE_LENTH,
               ROBOT_BASE_WIDTH);

ACTUATOR actuator[2][2];
#define M1 actuator[LEFT][FORWARD]
#define M2 actuator[RIGHT][FORWARD]
#define M3 actuator[LEFT][BACKWARD]
#define M4 actuator[RIGHT][BACKWARD]

float base_linear;
float base_angular;

unsigned long last_blink_time;
unsigned long joint_state_publish_last_time;

void M1_enc_f();
void M2_enc_f();
void M3_enc_f();
void M4_enc_f();

void setup()
{
    M1.setPins(M1_MOTOR_PIN_1, M1_MOTOR_PIN_2, M1_ENCODER_PIN);
    M1.setMaxRPM(MOTOR_MAX_RPM);
    M1.setTiksPerRevolution(MOTOR_TIKS_PER_REVOLUTION);
    M1.setMinPWMvalue(MIN_PWM_SIGNAL);
    M1.setPWMRanges(MIN_PWM, MAX_PWM);
    attachInterrupt(M1._enc_pin,M1_enc_f,MOTOR_ENCODER_READ_MODE);

    M2.setPins(M2_MOTOR_PIN_1, M2_MOTOR_PIN_2, M2_ENCODER_PIN);
    M2.setMaxRPM(MOTOR_MAX_RPM);
    M2.setTiksPerRevolution(MOTOR_TIKS_PER_REVOLUTION);
    M2.setMinPWMvalue(MIN_PWM_SIGNAL);
    M2.setPWMRanges(MIN_PWM, MAX_PWM);
    attachInterrupt(M2._enc_pin,M2_enc_f,MOTOR_ENCODER_READ_MODE);

    M3.setPins(M3_MOTOR_PIN_1, M3_MOTOR_PIN_2, M3_ENCODER_PIN);
    M3.setMaxRPM(MOTOR_MAX_RPM);
    M3.setTiksPerRevolution(MOTOR_TIKS_PER_REVOLUTION);
    M3.setMinPWMvalue(MIN_PWM_SIGNAL);
    M3.setPWMRanges(MIN_PWM, MAX_PWM);
    attachInterrupt(M3._enc_pin,M3_enc_f,MOTOR_ENCODER_READ_MODE);

    M4.setPins(M4_MOTOR_PIN_1, M4_MOTOR_PIN_2, M4_ENCODER_PIN);
    M4.setMaxRPM(MOTOR_MAX_RPM);
    M4.setTiksPerRevolution(MOTOR_TIKS_PER_REVOLUTION);
    M4.setMinPWMvalue(MIN_PWM_SIGNAL);
    M4.setPWMRanges(MIN_PWM, MAX_PWM);
    attachInterrupt(M4._enc_pin,M4_enc_f,MOTOR_ENCODER_READ_MODE);

    nh.initNode();
    nh.subscribe(cmd_vel_sub);
    nh.advertise(vels_pub);
}

void my_main()
{

    Kinematics::rpm rpm;
    rpm = kin.getRPM(base_linear, 0, base_angular);

    M1.writeRPM(rpm.motor1);
    M2.writeRPM(rpm.motor2);
    M3.writeRPM(rpm.motor4);
    M4.writeRPM(rpm.motor3);

    if (millis() - joint_state_publish_last_time > (1000.0 / VELOCITY_PUBLISH_RATE_Hz))
    {
        Kinematics::velocities vels;

        vels = kin.getVelocities(
            M1.getCurrentRPM(),
            M2.getCurrentRPM(),
            M4.getCurrentRPM(),
            M3.getCurrentRPM());

        actual_vels.linear.x = vels.linear_x;
        actual_vels.linear.y = vels.linear_y;
        actual_vels.angular.z = vels.angular_z;

        vels_pub.publish(&actual_vels);
        joint_state_publish_last_time = millis();
    }
}

void loop()
{
    if (nh.connected())
    {
        if (millis() - last_blink_time > 1000)
        {
            toggleLED();
            last_blink_time = millis();
        }
        my_main();
    }
    else
    {
        if (millis() - last_blink_time > 100)
        {
            toggleLED();
            last_blink_time = millis();
        }
        M1.writePWM(0);
        M2.writePWM(0);
        M3.writePWM(0);
        M4.writePWM(0);
    }

    M1.spin(MOTOR_FUNCTION_SPIN_PERIOD);
    M2.spin(MOTOR_FUNCTION_SPIN_PERIOD);
    M3.spin(MOTOR_FUNCTION_SPIN_PERIOD);
    M4.spin(MOTOR_FUNCTION_SPIN_PERIOD);
    // Serial3.println(" ");
    nh.spinOnce();
}

void cmd_vel_cb_f(const geometry_msgs::Twist &twist)
{
    base_linear = twist.linear.x;
    base_angular = twist.angular.z;
}

void M1_enc_f()
{
    if (M1.getDirection() == ROT_DIR_FORWARD)
    {
        M1._encoderTiks++;
    }
    else if (M1.getDirection() == ROT_DIR_BACKWARD)
    {
        M1._encoderTiks--;
    }
}
void M2_enc_f()
{
    if (M2.getDirection() == ROT_DIR_FORWARD)
    {
        M2._encoderTiks++;
    }
    else if (M2.getDirection() == ROT_DIR_BACKWARD)
    {
        M2._encoderTiks--;
    }
}
void M3_enc_f()
{
    if (M3.getDirection() == ROT_DIR_FORWARD)
    {
        M3._encoderTiks++;
    }
    else if (M3.getDirection() == ROT_DIR_BACKWARD)
    {
        M3._encoderTiks--;
    }
}
void M4_enc_f()
{
    if (M4.getDirection() == ROT_DIR_FORWARD)
    {
        M4._encoderTiks++;
    }
    else if (M4.getDirection() == ROT_DIR_BACKWARD)
    {
        M4._encoderTiks--;
    }
}