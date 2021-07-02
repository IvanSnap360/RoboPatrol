// #define MPU_ON
#define GPS1_ON
// #define GPS2_ON


#include <ros.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <arduino_functions.h>

#if defined(MPU_ON)
#include <mpu6050.h>
#endif

#if defined(GPS1_ON) || defined(GPS2_ON)
#include <GNSS.h>
#endif




#define MIN_PWM 0
#define MAX_PWM 65535
#define WHEEL_DEAMETR 1.0
#define BASE_WIDTH 1.0

ros::NodeHandle_<STM32Hardware, 25, 25, 2048, 2048> nh;

#if defined(MPU_ON)
geometry_msgs::Twist mpu_msg;
#endif

#if defined(GPS1_ON)
sensor_msgs::NavSatFix gps1_msg;
#endif
#if defined(GPS2_ON)
sensor_msgs::NavSatFix gps2_msg;
#endif

#if defined(MPU_ON)
uint32_t last_mpu_pub_time = 0;
#endif
#if defined(GPS1_ON)
uint32_t last_gps1_pub_time = 0;
#endif

#if defined(GPS2_ON)
uint32_t last_gps2_pub_time = 0;
#endif

void driver_cb(const geometry_msgs::Twist &msg);
#if defined(MPU_ON)
void mpu_publisher_f(uint32_t hz);
#endif

#if defined(GPS1_ON)
void gps1_publisher_f(uint32_t hz);
#endif

#if defined(GPS2_ON)
void gps2_publisher_f(uint32_t hz);
#endif

ros::Subscriber<geometry_msgs::Twist> driver_sub("/imu/driver_cmd", &driver_cb);
#if defined(MPU_ON)
ros::Publisher mpu_publisher("/imu/mpu", &mpu_msg);
#endif

#if defined(GPS1_ON)
ros::Publisher gps1_publisher("/imu/gps1", &gps1_msg);
#endif

#if defined(GPS2_ON)
ros::Publisher gps_publisher("/imu/gps2", &gps2_msg);
#endif

#if defined(MPU_ON)
MPU6050_t mpu_data;
#endif

#if defined(GPS1_ON)
GNSS_StateHandle GPS1_handle;
#endif

#if defined(GPS2_ON)
GNSS_StateHandle GPS2_handle;
#endif

float linear = 0;
float angular = 0;

void setup()
{
#if defined(MPU_ON)
    while (MPU6050_Init(&hi2c1) == 1)
    {
        TIM4->CCR4 = 65535;
    }
    TIM4->CCR4 = 0;
#endif

#if defined(GPS1_ON)
    GNSS_Init(&GPS1_handle, &huart2);
	HAL_Delay(1000);
	GNSS_LoadConfig(&GPS1_handle);
#endif

    nh.initNode();
    nh.subscribe(driver_sub);
#if defined(MPU_ON)
    nh.advertise(mpu_publisher);
#endif

#if defined(MPU_ON)
    last_mpu_pub_time = HAL_GetTick();
#endif
#if defined(GPS1_ON)
    last_gps1_pub_time = HAL_GetTick();
#endif
#if (GPS2_ON)
    uint32_t last_gps2_pub_time = HAL_GetTick();
#endif
}
void loop()
{

    if (nh.connected())
    {
        TIM4->CCR1 = 65535;
        TIM4->CCR3 = 0;

#if defined(MPU_ON)
        mpu_publisher_f(10.0);
#endif
#if defined(GPS1_ON)
        gps1_publisher_f(10.0);
#endif
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
#if defined(MPU_ON)
void mpu_publisher_f(uint32_t hz)
{
    if (HAL_GetTick() - last_mpu_pub_time > (uint32_t)1000 / hz)
    {
        // MPU6050_Read_All(&hi2c1, &mpu_data);
        MPU6050_Read_Gyro(&hi2c1, &mpu_data);
        MPU6050_Read_Accel(&hi2c1, &mpu_data);
        mpu_msg.linear.x = roundl(mpu_data.Ax);
        mpu_msg.linear.y = roundl(mpu_data.Ay);
        mpu_msg.linear.z = roundl(mpu_data.Az);

        mpu_msg.angular.x = roundl(mpu_data.Gx);
        mpu_msg.angular.y = roundl(mpu_data.Gy);
        mpu_msg.angular.z = roundl(mpu_data.Gz);

        mpu_publisher.publish(&mpu_msg);
        last_mpu_pub_time = HAL_GetTick();
    }
}
#endif

#if defined(GPS1_ON)
void gps1_publisher_f(uint32_t hz)
{
    if (HAL_GetTick() - last_gps1_pub_time > (uint32_t)1000 / hz)
    {
        gps1_msg.altitude = 1;
        gps1_msg.latitude = 2;
        gps1_msg.longitude = 3;
        gps1_publisher.publish(&gps1_msg);
        last_gps1_pub_time = HAL_GetTick();
    }
}
#endif

#if (GPS2_ON)
void gps2_publisher_f(uint32_t hz)
{
    gps2_msg.altitude = 1;
    gps2_msg.latitude = 2;
    gps2_msg.longitude = 3;
    gps2_publisher.publish(&gps1_msg);
}
#endif