#include "config.h"
#include "actuator.h"
#include <ros.h>
#include <sensor_msgs/JointState.h>

ACTUATOR actuator[2][2];
#define M1 actuator[LEFT][FORWARD]
#define M2 actuator[RIGHT][FORWARD]
#define M3 actuator[LEFT][BACKWARD]
#define M4 actuator[RIGHT][BACKWARD]

ros::NodeHandle_<ROS_HARDWARE, ROS_MAX_SUBSCRIBERS, ROS_MAX_PUBLISHERS, ROS_MAX_INPUT_BUFFER_SIZE, ROS_MAX_OUTPUT_BUFFER_SIZE> nh;

void joint_control_cb_f(const sensor_msgs::JointState &joint_data)
{
    M1.writeAngularVelocity(joint_data.velocity[0]);
    M2.writeAngularVelocity(joint_data.velocity[1]);
    M3.writeAngularVelocity(joint_data.velocity[2]);
    M4.writeAngularVelocity(joint_data.velocity[3]);
}

ros::Subscriber<sensor_msgs::JointState> joint_control_sub("/joint_contol", &joint_control_cb_f);

sensor_msgs::JointState state_msg;
ros::Publisher joint_state_pub("/joint_state", &state_msg);

char *state_names[MOTOR_COUNT] = {"M1", "M2", "M3", "M4"};
float state_pos[MOTOR_COUNT] = {0.0, 0.0, 0.0, 0.0};
float state_lpos[MOTOR_COUNT] = {0.0, 0.0, 0.0, 0.0};
float state_vel[MOTOR_COUNT] = {0.0, 0.0, 0.0, 0.0};
float state_eff[MOTOR_COUNT] = {0.0, 0.0, 0.0, 0.0};

void M1_enc_f();
void M2_enc_f();
void M3_enc_f();
void M4_enc_f();

unsigned long jont_state_last_publish_time = 0;

void setup()
{

    M1.setPins(M1_MOTOR_PIN_1, M1_MOTOR_PIN_2, M1_ENCODER_PIN);
    M1.setWheelDeametr(ROBOT_WHEEL_DEAMETR);
    M1.setTiksPerRevolution(MOTOR_TIKS_PER_REVOLUTION);
    M1.setMinPWMvalue(MIN_PWM_SIGNAL);
    M1.setPWMRanges(MIN_PWM, MAX_PWM);
    attachInterrupt(M1._enc_pin, M1_enc_f, MOTOR_ENCODER_READ_MODE);

    M2.setPins(M2_MOTOR_PIN_1, M2_MOTOR_PIN_2, M2_ENCODER_PIN);
    M2.setWheelDeametr(ROBOT_WHEEL_DEAMETR);
    M2.setTiksPerRevolution(MOTOR_TIKS_PER_REVOLUTION);
    M2.setMinPWMvalue(MIN_PWM_SIGNAL);
    M2.setPWMRanges(MIN_PWM, MAX_PWM);
    attachInterrupt(M2._enc_pin, M2_enc_f, MOTOR_ENCODER_READ_MODE);

    M3.setPins(M3_MOTOR_PIN_1, M3_MOTOR_PIN_2, M3_ENCODER_PIN);
    M3.setWheelDeametr(ROBOT_WHEEL_DEAMETR);
    M3.setTiksPerRevolution(MOTOR_TIKS_PER_REVOLUTION);
    M3.setMinPWMvalue(MIN_PWM_SIGNAL);
    M3.setPWMRanges(MIN_PWM, MAX_PWM);
    attachInterrupt(M3._enc_pin, M3_enc_f, MOTOR_ENCODER_READ_MODE);

    M4.setPins(M4_MOTOR_PIN_1, M4_MOTOR_PIN_2, M4_ENCODER_PIN);
    M4.setWheelDeametr(ROBOT_WHEEL_DEAMETR);
    M4.setTiksPerRevolution(MOTOR_TIKS_PER_REVOLUTION);
    M4.setMinPWMvalue(MIN_PWM_SIGNAL);
    M4.setPWMRanges(MIN_PWM, MAX_PWM);
    attachInterrupt(M4._enc_pin, M4_enc_f, MOTOR_ENCODER_READ_MODE);

    state_msg.header.frame_id = "/driver_states";
    state_msg.name_length = MOTOR_COUNT;
    state_msg.velocity_length = MOTOR_COUNT;
    state_msg.position_length = MOTOR_COUNT;
    state_msg.effort_length = MOTOR_COUNT;
    state_msg.name = state_names;
    state_msg.position = state_pos;
    state_msg.velocity = state_vel;
    state_msg.effort = state_eff;

    nh.initNode();
    nh.advertise(joint_state_pub);
    nh.subscribe(joint_control_sub);
}

unsigned long last_blink_time;

void loop()
{
    if (nh.connected())
    {
        if (millis() - last_blink_time > 1000)
        {
            toggleLED();
            last_blink_time = millis();
        }
        unsigned long t = millis() - jont_state_last_publish_time;
        if (t > (1000.0 / JOINTS_STATE_PUBLISH_RATE_HZ))
        {
            state_pos[0] = M1.impulse2meters(M1.abs_tiks);
            state_pos[1] = M2.impulse2meters(M2.abs_tiks);
            state_pos[2] = M3.impulse2meters(M3.abs_tiks);
            state_pos[3] = M4.impulse2meters(M4.abs_tiks);

            state_vel[0] = M1.impulse2rad(M1.rel_tiks / ((float)t / 1000.0));
            state_vel[1] = M2.impulse2rad(M2.rel_tiks / ((float)t / 1000.0));
            state_vel[2] = M3.impulse2rad(M3.rel_tiks / ((float)t / 1000.0));
            state_vel[3] = M4.impulse2rad(M4.rel_tiks / ((float)t / 1000.0));

            state_msg.header.stamp = nh.now();

            joint_state_pub.publish(&state_msg);

            jont_state_last_publish_time = millis();

            M1.rel_tiks = 0;
            M2.rel_tiks = 0;
            M3.rel_tiks = 0;
            M4.rel_tiks = 0;
        }
        // my_main();
    }
    else
    {
        if (millis() - last_blink_time > 100)
        {
            toggleLED();
            last_blink_time = millis();
        }
        M1.writeAngularVelocity(0.0);
        M2.writeAngularVelocity(0.0);
        M3.writeAngularVelocity(0.0);
        M4.writeAngularVelocity(0.0);
    }
    M1.spin(10);
    M2.spin(10);
    M3.spin(10);
    M4.spin(10);
    nh.spinOnce();
}

void M1_enc_f()
{
    if (M1.getDirection() > 0)
    {
        M1.rel_tiks++;
    }
    else if (M1.getDirection() < 0)
    {
        M1.rel_tiks--;
    }
    M1.abs_tiks++;
}
void M2_enc_f()
{
    if (M2.getDirection() > 0)
    {
        M2.rel_tiks++;
    }
    else if (M2.getDirection() < 0)
    {
        M2.rel_tiks--;
    }
    M2.abs_tiks++;
}
void M3_enc_f()
{
    if (M3.getDirection() > 0)
    {
        M3.rel_tiks++;
    }
    else if (M3.getDirection() < 0)
    {
        M3.rel_tiks--;
    }
    M3.abs_tiks++;
}
void M4_enc_f()
{
    if (M4.getDirection() > 0)
    {
        M4.rel_tiks++;
    }
    else if (M4.getDirection() < 0)
    {
        M4.rel_tiks--;
    }
    M4.abs_tiks++;
}
