#include <ros.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

ros::NodeHandle_<ArduinoHardware, 25, 25, 2048, 2048> nh;
// выводы к которым подключен драйвер моторов
#define LEFT_MOTOR_PIN_1 2
#define LEFT_MOTOR_PIN_2 3
#define RIGHT_MOTOR_PIN_1 4
#define RIGHT_MOTOR_PIN_2 5

#define LEFT_ENCODER_INTERRUPT_NB 16  // номер прерывания
#define RIGHT_ENCODER_INTERRUPT_NB 18 // номер прерывания

#define Kp 120.0 // пропорциональный коэффициент для ПИД регулятора (41.7)
#define Ki 0.1   // интегральный коэффициент для ПИД регулятора
#define Kd 0.2   // дифференциальный коэффициент для ПИД регулятора

#define MOTOR_VALUE_MAX 255 // максимальное значение подаваемое на драйвер
#define MOTOR_VALUE_MIN 50  // минимальное значение подаваемое на драйвер

#define WHEEL_BASE 0.184           // база колесная в метрах
#define WHEEL_DIAMETER 0.084       // диаметр колеса в метрах
#define WHEEL_IMPULSE_COUNT 1225.0 // количество импульсов на оборот колеса

#define COUNT_MOTORS 2 // количество моторов

// стороны робота
#define LEFT 0
#define RIGHT 1

unsigned long last_time_pub; // время последней публикации

float linear = 0.0;
float angular = 0.0;

float enc_count[COUNT_MOTORS] = {0.0, 0.0};    // счетчик для левого колеса
float speed_wheel[COUNT_MOTORS] = {0.0, 0.0};  // скорость моторов требуемая
float speed_actual[COUNT_MOTORS] = {0.0, 0.0}; // скорость моторов реальная

float e_prev[COUNT_MOTORS] = {0.0, 0.0}; //последнее значение разницы скорости движения
float I_prev[COUNT_MOTORS] = {0.0, 0.0}; //последние значения выборки интегральной составляющей ПИД регулятора

void cmd_vel_cb(const geometry_msgs::Twist &vel)
{
  linear = vel.linear.x;
  angular = vel.angular.z;
}

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/cmd_vel", &cmd_vel_cb);

geometry_msgs::TransformStamped odom_tf;
tf::TransformBroadcaster tf_broadcaster;

nav_msgs::Odometry odom;
ros::Publisher odom_pub("/odometry", &odom);

void setup()
{
  // инициализация выходов на драйверы управления моторами
  pinMode(LEFT_MOTOR_PIN_1, OUTPUT);
  pinMode(LEFT_MOTOR_PIN_2, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN_1, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN_2, OUTPUT);

  // инициализация прерываний
  attachInterrupt(LEFT_ENCODER_INTERRUPT_NB, callBackInterruptLeftEncoder, CHANGE);
  attachInterrupt(RIGHT_ENCODER_INTERRUPT_NB, callBackInterruptRightEncoder, CHANGE);

  nh.getHardware()->setBaud(1000000);
  nh.initNode();
  nh.subscribe(cmd_vel_sub);

  
}

void my_main()
{
  unsigned long t = millis() - last_time_pub;

  if (t > 100)
  {
    kinematic_solver(t);
  }
}

void kinematic_solver(unsigned long t)
{
  float V = linear;               //линейная скорость
  float W = angular;              //угловая скорость
  float r = WHEEL_DIAMETER / 2.0; //радиус колеса
  float L = WHEEL_BASE;           //база робота

  // вычисление требуемой скорости вращения колес (рад/с)
  speed_wheel[LEFT] = (1.0 / r) * V - (L / r) * W;
  speed_wheel[RIGHT] = (1.0 / r) * V + (L / r) * W;

  //вычисление текущей скорости вращения колес
  speed_actual[LEFT] = impulse2rad(enc_count[LEFT]) / ((float)t/ 1000.0);
  speed_actual[RIGHT] = impulse2rad(enc_count[RIGHT]) / ((float)t / 1000.0);

  //вычисление значения для драйвера используя ПИД-регулятор
  float pid_left = motorsPID(speed_wheel[LEFT], speed_actual[LEFT], LEFT);
  float pid_right = motorsPID(speed_wheel[RIGHT], speed_actual[RIGHT], RIGHT);

  moveMotor(pid_left, LEFT);
  moveMotor(pid_right, RIGHT);

  enc_count[LEFT] = 0.0;
  enc_count[RIGHT] = 0.0;

  last_time_pub = millis(); // фиксируем время последней публикации
}

// ПИД-регулятор
int motorsPID(float speed_control, float speed_actual, int side)
{
  // при управляющем воздействии равным нулю фиксируем составляющие на текущем шаге и возвращаем управляющее возжействие равным нулю
  if (speed_control == 0.0)
  {
    I_prev[side] = 0.0;
    e_prev[side] = 0.0;
    return 0;
  }

  // расчет ошибки между требуемой скоростью и фактической
  float e = speed_control - speed_actual; //разница в скорости текущая в m/s и желаемая m/s

  // ПИД регулятор для рассчета значения для драйвера моторов
  float P = Kp * e;
  float I = I_prev[side] + Ki * e;
  float D = Kd * (e - e_prev[side]);
  float value = round(P + I + D);

  I_prev[side] = I; //фиксируем интегральную составляющую
  e_prev[side] = e; //фиксируем последнее значение разницы в скорости

  return value;
}

// управление мотором на определенной стороне робота
void moveMotor(int value, int side)
{
  // избавляемся от переполнения ШИМ
  if (value > MOTOR_VALUE_MAX)
    value = MOTOR_VALUE_MAX;
  if (value < -MOTOR_VALUE_MAX)
    value = -MOTOR_VALUE_MAX;

  // убираем значения ниже минимального значения при котором моторы могут вращаться
  if (value < 0 && value >= -MOTOR_VALUE_MIN)
    value = -MOTOR_VALUE_MIN;
  if (value > 0 && value <= MOTOR_VALUE_MIN)
    value = MOTOR_VALUE_MIN;

  // определяем направление вращения и передаем значения на драйвер
  if (value >= 0)
  {
    if (value == 0)
    {
      // стоп мотор
      analogWrite(side == LEFT ? LEFT_MOTOR_PIN_1 : RIGHT_MOTOR_PIN_1, 0);
      analogWrite(side == LEFT ? LEFT_MOTOR_PIN_2 : RIGHT_MOTOR_PIN_2, 0);
    }
    else
    {
      // вращение вперед
      analogWrite(side == LEFT ? LEFT_MOTOR_PIN_1 : RIGHT_MOTOR_PIN_1, abs(value));
      analogWrite(side == LEFT ? LEFT_MOTOR_PIN_2 : RIGHT_MOTOR_PIN_2, 0);
    }
  }
  else
  {
    // вращение назад
    analogWrite(side == LEFT ? LEFT_MOTOR_PIN_1 : RIGHT_MOTOR_PIN_1, 0);
    analogWrite(side == LEFT ? LEFT_MOTOR_PIN_2 : RIGHT_MOTOR_PIN_2, abs(value));
  }
}

// получаем направление вращения двигателя относительно значения скорости вращения
float getRotationDir(float value)
{
  if (value >= 0)
  {
    if (value == 0)
    {
      return 0.0;
    }
    else
    {
      return 1.0;
    }
  }
  else
  {
    return -1.0;
  }
}

// обработчик прерывания для левого колеса
inline void callBackInterruptLeftEncoder()
{
  enc_count[LEFT] += getRotationDir(speed_wheel[LEFT]);
}

// обработчик прерывания для правого колеса
inline void callBackInterruptRightEncoder()
{
  enc_count[RIGHT] += getRotationDir(speed_wheel[RIGHT]);
}

// преобразование импульсов в метры
inline float impulse2meters(float x)
{
  return (x / WHEEL_IMPULSE_COUNT) * M_PI * WHEEL_DIAMETER;
}

// преобразование импульсов в радианы
inline float impulse2rad(float x)
{
  return (x / WHEEL_IMPULSE_COUNT) * 2.0 * M_PI;
}

void loop()
{
  if (nh.connected())
  {
    my_main();
  }
  else
  {
    toggleLED();
    delay(100);
  }
  nh.spinOnce();
}