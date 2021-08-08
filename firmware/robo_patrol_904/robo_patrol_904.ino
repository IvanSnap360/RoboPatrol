#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/UInt32.h>
#define DEBUG

#define MAX_PWM 255
#define MIN_PWM -255

#define MOTOR_COUNT 2
#define ENCODER_COUNT 2

#define WHEEL_DEAMETR 0.084
#define TIKS_PER_REVOLUTION 1225
#define BASE_WIDTH 0.184

#define FORWARD 1   // команда вперед
#define BACKWARD -1 // команда назад
#define STOP 0

// M1 --> LEFT
// M2 --> RIGHT

#define M1_FW_PIN 2
#define M1_BW_PIN 3

#define M2_FW_PIN 4
#define M2_BW_PIN 5

#define M1_ENC_A 16
#define M1_ENC_B 17

#define M2_ENC_A 18
#define M2_ENC_B 19

#define RIGHT 1
#define LEFT 0

#define ENC_A 0
#define ENC_B 1

#define PREV 0
#define NOW 1

long enc_counter[MOTOR_COUNT][2];
unsigned long simple_enc_counter[MOTOR_COUNT];
bool sign[MOTOR_COUNT][ENCODER_COUNT];
float speed_wheel[MOTOR_COUNT];

long enc_delta[MOTOR_COUNT];
long lenth_delta[MOTOR_COUNT];

float lin = 0.0;
float ang = 0.0;

void left_enc_A();
void left_enc_B();
void right_enc_A();
void right_enc_B();

void M1_ENC_CB()
{
    left_enc_A();
    left_enc_B();
}
void M2_ENC_CB()
{
    right_enc_A();
    right_enc_B();
}

void vel_cb(const geometry_msgs::Twist &data)
{
    lin = data.linear.x;
    ang = data.angular.z;
}

ros::NodeHandle_<ArduinoHardware, 25, 25, 2048, 2048> nh;
ros::Subscriber<geometry_msgs::Twist> vel_sub("/cmd_vel", &vel_cb);

unsigned long odom_solv_last_time = 0.0;

geometry_msgs::Pose base_link;
ros::Publisher pos_pub("/base_link/position", &base_link);
void setup()
{
#if defined(DEBUG)
    Serial1.setDxlMode(false);
    Serial1.begin(9600);
#endif
    // motors pins setup
    pinMode(M1_FW_PIN, OUTPUT);
    pinMode(M1_BW_PIN, OUTPUT);
    pinMode(M2_FW_PIN, OUTPUT);
    pinMode(M2_BW_PIN, OUTPUT);

    //encoders pins ssetup
    pinMode(M1_ENC_B, INPUT_PULLDOWN);
    pinMode(M1_ENC_A, INPUT_PULLDOWN);
    pinMode(M2_ENC_A, INPUT_PULLDOWN);
    pinMode(M2_ENC_B, INPUT_PULLDOWN);

    attachInterrupt(M1_ENC_A, M1_ENC_CB, CHANGE);
    attachInterrupt(M2_ENC_A, M2_ENC_CB, CHANGE);

    nh.getHardware()->setBaud(1000000);
    nh.initNode();
    nh.subscribe(vel_sub);
    nh.advertise(pos_pub);
}
void loop()
{
    if (nh.connected())
    {
        float V = -lin;                //линейная скорость
        float W = ang;                 //угловая скорость
        float r = WHEEL_DEAMETR / 2.0; //радиус колеса
        float L = BASE_WIDTH;          //база робота

        // вычисление требуемой скорости вращения колес (рад/с)
        speed_wheel[LEFT] = r * ((1.0 / r) * V - (L / r) * W);
        speed_wheel[RIGHT] = r * ((1.0 / r) * V + (L / r) * W);

        motorLEFTrun(getMotorValue(speed_wheel[LEFT]), getRotationDir(speed_wheel[LEFT]));
        motorRIGHTrun(getMotorValue(speed_wheel[RIGHT]), getRotationDir(speed_wheel[RIGHT]));

        if (millis() - odom_solv_last_time > 10)
        {
            enc_delta[LEFT] = enc_counter[LEFT][NOW] - enc_counter[LEFT][PREV];
            enc_delta[RIGHT] = enc_counter[RIGHT][NOW] - enc_counter[RIGHT][PREV];

            lenth_delta[LEFT] = tiks2meters(enc_delta[LEFT]);
            lenth_delta[RIGHT] = tiks2meters(enc_delta[RIGHT]);

            long base_lenth_delta = (lenth_delta[LEFT] + lenth_delta[RIGHT]) / 2;
            long base_theta_delta = (lenth_delta[RIGHT] - lenth_delta[LEFT]) / BASE_WIDTH;

            base_link.position.x += cos(base_link.orientation.z + (base_theta_delta / 2));
            base_link.position.y += sin(base_link.orientation.z + (base_theta_delta / 2));
            base_link.orientation.z += base_theta_delta;

            enc_counter[LEFT][PREV] = enc_counter[LEFT][NOW];
            enc_counter[RIGHT][PREV] = enc_counter[RIGHT][NOW];

            pos_pub.publish(&base_link);
            odom_solv_last_time = millis();
        }
#if defined(DEBUG)
        Serial1.print("enc_counter[LEFT][NOW]: ");
        Serial1.print(enc_counter[LEFT][NOW]);
        Serial1.print("  enc_counter[LEFT][PREV]: ");
        Serial1.print(enc_counter[LEFT][PREV]);
        Serial1.print("  enc_counter[RIGHT][NOW]: ");
        Serial1.print(enc_counter[RIGHT][NOW]);
        Serial1.print("  enc_counter[RIGHT][PREV]: ");
        Serial1.println(enc_counter[RIGHT][PREV]);
#endif
    }
    else
    {
        toggleLED();
        delay(100);
    }
    nh.spinOnce();
}

void right_enc_A()
{
    sign[RIGHT][ENC_A] = digitalRead(M2_ENC_A);
    if (sign[RIGHT][ENC_B] != sign[RIGHT][ENC_A])
    {
        enc_counter[RIGHT][NOW]++;
    }
    else
    {
        enc_counter[RIGHT][NOW]--;
    }
    simple_enc_counter[RIGHT]++;
}

void right_enc_B()
{
    sign[RIGHT][ENC_B] = digitalRead(M2_ENC_B);
    if (sign[RIGHT][ENC_B] == sign[RIGHT][ENC_A])
    {
        enc_counter[RIGHT][NOW]++;
    }
    else
    {
        enc_counter[RIGHT][NOW]--;
    }
}

void left_enc_A()
{
    sign[LEFT][ENC_A] = digitalRead(M1_ENC_A);
    if (sign[LEFT][ENC_B] == sign[LEFT][ENC_A])
    {
        enc_counter[LEFT][NOW]++;
    }
    else
    {
        enc_counter[LEFT][NOW]--;
    }
    simple_enc_counter[LEFT]++;
}

void left_enc_B()
{
    sign[LEFT][ENC_B] = digitalRead(M1_ENC_B);
    if (sign[LEFT][ENC_B] != sign[LEFT][ENC_A])
    {
        enc_counter[LEFT][NOW]++;
    }
    else
    {
        enc_counter[LEFT][NOW]--;
    }
}

void motorLEFTrun(int pwms, int dir)
{

    if (dir == FORWARD)
    {
        analogWrite(M2_FW_PIN, 0);
        analogWrite(M2_BW_PIN, pwms);
    }
    else if (dir == BACKWARD)
    {
        analogWrite(M2_FW_PIN, -pwms);
        analogWrite(M2_BW_PIN, 0);
    }
    else if (dir == STOP)
    {
        analogWrite(M2_FW_PIN, 0);
        analogWrite(M2_BW_PIN, 0);
    }
}

void motorRIGHTrun(int pwms, int dir)
{

    if (dir == FORWARD)
    {
        analogWrite(M1_FW_PIN, 0);
        analogWrite(M1_BW_PIN, pwms);
    }
    else if (dir == BACKWARD)
    {
        analogWrite(M1_FW_PIN, -pwms);
        analogWrite(M1_BW_PIN, 0);
    }
    else if (dir == STOP)
    {
        analogWrite(M1_FW_PIN, 0);
        analogWrite(M1_BW_PIN, 0);
    }
}

int getMotorValue(float value)
{
    return map(value * 1000, -1000, 1000, MIN_PWM, MAX_PWM);
}

int getRotationDir(float value)
{
    if (value >= 0)
    {
        if (value == 0)
        {
            return STOP;
        }
        else
        {
            return FORWARD;
        }
    }
    else
    {
        return BACKWARD;
    }
}

float tiks2meters(long value)
{
    return PI * WHEEL_DEAMETR * (value / TIKS_PER_REVOLUTION);
}

float tiks2rad(float x)
{
    return (x / TIKS_PER_REVOLUTION) * 2.0 * PI;
}