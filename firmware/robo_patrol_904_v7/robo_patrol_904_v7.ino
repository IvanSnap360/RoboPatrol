#include <ros.h>
#include <ros/time.h>
#include "actuator.h"
#include "config.h"
#include <sensor_msgs/JointState.h>

ros::NodeHandle_<ROS_HARDWARE,
                 ROS_MAX_SUBSCRIBERS,
                 ROS_MAX_PUBLISHERS,
                 ROS_MAX_INPUT_BUFFER_SIZE,
                 ROS_MAX_OUTPUT_BUFFER_SIZE>
    nh;
sensor_msgs::JointState state_msg;
ros::Publisher state_pub("/joint_states", &state_msg);

void joint_control_cb_f(const sensor_msgs::JointState &msg);
ros::Subscriber<sensor_msgs::JointState> joints_sub("/joint_control", &joint_control_cb_f);

double x = 0.0;
double y = 0.0;
double th = 0.0;

ACTUATOR actuator[2][2];
#define M1 actuator[LEFT][FORWARD]
#define M2 actuator[RIGHT][FORWARD]
#define M3 actuator[LEFT][BACKWARD]
#define M4 actuator[RIGHT][BACKWARD]

char *state_names[MOTOR_COUNT] = {"M1", "M2", "M3", "M4"};
float state_pos[MOTOR_COUNT] = {0.0, 0.0, 0.0, 0.0};
float state_lpos[MOTOR_COUNT] = {0.0, 0.0, 0.0, 0.0};
float state_vel[MOTOR_COUNT] = {0.0, 0.0, 0.0, 0.0};
float state_eff[MOTOR_COUNT] = {0.0, 0.0, 0.0, 0.0};

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
    attachInterrupt(M1._enc_pin, M1_enc_f, MOTOR_ENCODER_READ_MODE);

    M2.setPins(M2_MOTOR_PIN_1, M2_MOTOR_PIN_2, M2_ENCODER_PIN);
    M2.setMaxRPM(MOTOR_MAX_RPM);
    M2.setTiksPerRevolution(MOTOR_TIKS_PER_REVOLUTION);
    M2.setMinPWMvalue(MIN_PWM_SIGNAL);
    M2.setPWMRanges(MIN_PWM, MAX_PWM);
    attachInterrupt(M2._enc_pin, M2_enc_f, MOTOR_ENCODER_READ_MODE);

    M3.setPins(M3_MOTOR_PIN_1, M3_MOTOR_PIN_2, M3_ENCODER_PIN);
    M3.setMaxRPM(MOTOR_MAX_RPM);
    M3.setTiksPerRevolution(MOTOR_TIKS_PER_REVOLUTION);
    M3.setMinPWMvalue(MIN_PWM_SIGNAL);
    M3.setPWMRanges(MIN_PWM, MAX_PWM);
    attachInterrupt(M3._enc_pin, M3_enc_f, MOTOR_ENCODER_READ_MODE);

    M4.setPins(M4_MOTOR_PIN_1, M4_MOTOR_PIN_2, M4_ENCODER_PIN);
    M4.setMaxRPM(MOTOR_MAX_RPM);
    M4.setTiksPerRevolution(MOTOR_TIKS_PER_REVOLUTION);
    M4.setMinPWMvalue(MIN_PWM_SIGNAL);
    M4.setPWMRanges(MIN_PWM, MAX_PWM);
    attachInterrupt(M4._enc_pin, M4_enc_f, MOTOR_ENCODER_READ_MODE);

    nh.initNode();
    nh.subscribe(joints_sub);
    nh.advertise(state_pub);

    state_msg.header.frame_id = "/driver_states";
    state_msg.name_length = MOTOR_COUNT;
    state_msg.velocity_length = MOTOR_COUNT;
    state_msg.position_length = MOTOR_COUNT;
    state_msg.effort_length = MOTOR_COUNT;
    state_msg.name = state_names;
    state_msg.position = state_pos;
    state_msg.velocity = state_vel;
    state_msg.effort = state_eff;
}

void my_main()
{
    if (millis() - joint_state_publish_last_time > (1000.0 / VELOCITY_PUBLISH_RATE_Hz))
    {
        unsigned long t = millis() - joint_state_publish_last_time;
        state_pos[0] = impulse2meters(M1._encoderTiks2);
        state_pos[1] = impulse2meters(M2._encoderTiks2);
        state_pos[2] = impulse2meters(M3._encoderTiks2);
        state_pos[3] = impulse2meters(M4._encoderTiks2);

        state_vel[0] = M1.getCurrentRPM();
        state_vel[1] = M2.getCurrentRPM();
        state_vel[2] = M3.getCurrentRPM();
        state_vel[3] = M4.getCurrentRPM();

        state_msg.header.stamp = nh.now();

        state_pub.publish(&state_msg);

        M1._encoderTiks = 0;
        M2._encoderTiks = 0;
        M3._encoderTiks = 0;
        M4._encoderTiks = 0;

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

void joint_control_cb_f(const sensor_msgs::JointState &msg)
{
    M1.writeRPM(msg.velocity[0]);
    M2.writeRPM(msg.velocity[1]);
    M3.writeRPM(msg.velocity[2]);
    M4.writeRPM(msg.velocity[3]);
}

void M1_enc_f()
{
    if (M1.getDirection() == ROT_DIR_FORWARD)
    {
        M1._encoderTiks++;
        M1._encoderTiks2++;
    }
    else if (M1.getDirection() == ROT_DIR_BACKWARD)
    {
        M1._encoderTiks--;
        M1._encoderTiks2--;
    }
}
void M2_enc_f()
{
    if (M2.getDirection() == ROT_DIR_FORWARD)
    {
        M2._encoderTiks++;
        M2._encoderTiks2++;
    }
    else if (M2.getDirection() == ROT_DIR_BACKWARD)
    {
        M2._encoderTiks--;
        M2._encoderTiks2--;
    }
}
void M3_enc_f()
{
    if (M3.getDirection() == ROT_DIR_FORWARD)
    {
        M3._encoderTiks++;
        M3._encoderTiks2++;
    }
    else if (M3.getDirection() == ROT_DIR_BACKWARD)
    {
        M3._encoderTiks--;
        M3._encoderTiks2--;
    }
}
void M4_enc_f()
{
    if (M4.getDirection() == ROT_DIR_FORWARD)
    {
        M4._encoderTiks++;
        M4._encoderTiks2++;
    }
    else if (M4.getDirection() == ROT_DIR_BACKWARD)
    {
        M4._encoderTiks--;
        M4._encoderTiks2--;
    }
    
}
inline float impulse2meters(float x)
{
    return (x / MOTOR_TIKS_PER_REVOLUTION) * M_PI * ROBOT_WHEEL_DEAMETR;
}