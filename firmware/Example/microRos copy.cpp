#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <AS5600.h>

#define USE_MPU6050_IMU
#include "imu.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>
#include <sensor_msgs/msg/imu.h>

#define TCAADDR 0x70

#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      rclErrorLoop();            \
    }                            \
  }
#define RCSOFTCHECK(fn)          \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
    }                            \
  }
#define EXECUTE_EVERY_N_MS(MS, X)      \
  do                                   \
  {                                    \
    static volatile int64_t init = -1; \
    if (init == -1)                    \
    {                                  \
      init = uxr_millis();             \
    }                                  \
    if (uxr_millis() - init > MS)      \
    {                                  \
      X;                               \
      init = uxr_millis();             \
    }                                  \
  } while (0)

//------------------------------ < Define > -------------------------------------//

IPAddress agent_ip(192, 168, 2, 34);
char ssid[] = "REAI_ROBOT_2.4G";
char psk[] = "reaicmubot";

rcl_publisher_t encoder_publisher;
rcl_publisher_t limit_publisher;
rcl_publisher_t imu1_publisher;
rcl_publisher_t imu2_publisher;

sensor_msgs__msg__Imu imu1_msg;
sensor_msgs__msg__Imu imu2_msg;
std_msgs__msg__Float32 encoder_msg;
std_msgs__msg__Bool limit_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;
rcl_init_options_t init_options;

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_update = 0;

enum states
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

IMU imu1;
IMU imu2;

AS5600 as5600;

//------------------------------ < Fuction Prototype > ------------------------------//

void flashLED(int n_times);
void rclErrorLoop();
void syncTime();
void readSensor();
void publishData();
bool createEntities();
bool destroyEntities();
struct timespec getTime();
void tcaSelect(uint8_t i);

//------------------------------ < Main > -------------------------------------//

void setup()
{
  pinMode(19, INPUT);

  Wire.begin();
  Serial.begin(115200);
  // set_microros_wifi_transports(ssid, psk, agent_ip, 8888);
  set_microros_serial_transports(Serial);

  tcaSelect(0);
  bool imu_ok1 = imu1.init();
  if (!imu_ok1)
  {
    while (1)
    {
      flashLED(3);
    }
  }

  tcaSelect(1); // Select channel 1
  bool imu_ok2 = imu2.init();
  if (!imu_ok2)
  {
    while (1)
    {
      flashLED(3);
    }
  }

  tcaSelect(2);
  as5600.isConnected();
}

void loop()
{
  switch (state)
  {
  case WAITING_AGENT:
    EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
    break;
  case AGENT_AVAILABLE:
    state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
    if (state == WAITING_AGENT)
    {
      destroyEntities();
    }
    break;
  case AGENT_CONNECTED:
    EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
    if (state == AGENT_CONNECTED)
    {
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    break;
  case AGENT_DISCONNECTED:
    destroyEntities();
    state = WAITING_AGENT;
    break;
  default:
    break;
  }
}

//------------------------------ < Fuction > -------------------------------------//

void controlCallback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    readSensor();
    publishData();
  }
}

bool createEntities()
{
  allocator = rcl_get_default_allocator();

  init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_init_options_set_domain_id(&init_options, 10);

  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

  // create node
  RCCHECK(rclc_node_init_default(&node, "sensor_esp32", "", &support));

  // create IMU publisher
  RCCHECK(rclc_publisher_init_default(
      &imu1_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      "imu1/data"));
  RCCHECK(rclc_publisher_init_default(
      &imu2_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      "imu2/data"));
  RCCHECK(rclc_publisher_init_default(
      &limit_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "limit/data"));
  RCCHECK(rclc_publisher_init_default(
      &encoder_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "encoder/data"));

  // create timer for actuating the motors at 50 Hz (1000/20)
  const unsigned int control_timeout = 20;
  RCCHECK(rclc_timer_init_default(
      &control_timer,
      &support,
      RCL_MS_TO_NS(control_timeout),
      controlCallback));
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

  // synchronize time with the agent
  syncTime();

  return true;
}

bool destroyEntities()
{
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&imu1_publisher, &node);
  rcl_publisher_fini(&imu2_publisher, &node);
  rcl_publisher_fini(&limit_publisher, &node);
  rcl_publisher_fini(&encoder_publisher, &node);
  rcl_node_fini(&node);
  rcl_timer_fini(&control_timer);
  rclc_executor_fini(&executor);
  rclc_support_fini(&support);

  return true;
}

void readSensor()
{
  tcaSelect(0);
  imu1_msg = imu1.getData();

  tcaSelect(1);
  imu2_msg = imu2.getData();

  tcaSelect(2);
  encoder_msg.data = as5600.rawAngle() * AS5600_RAW_TO_DEGREES;

  limit_msg.data = (bool)digitalRead(19);
}

void publishData()
{

  struct timespec time_stamp = getTime();

  imu1_msg.header.stamp.sec = time_stamp.tv_sec;
  imu1_msg.header.stamp.nanosec = time_stamp.tv_nsec;

  imu2_msg.header.stamp.sec = time_stamp.tv_sec;
  imu2_msg.header.stamp.nanosec = time_stamp.tv_nsec;

  RCSOFTCHECK(rcl_publish(&imu1_publisher, &imu1_msg, NULL));
  RCSOFTCHECK(rcl_publish(&imu2_publisher, &imu2_msg, NULL));
  RCSOFTCHECK(rcl_publish(&limit_publisher, &limit_msg, NULL));
  RCSOFTCHECK(rcl_publish(&encoder_publisher, &encoder_msg, NULL));
}

void syncTime()
{
  // get the current time from the agent
  unsigned long now = millis();
  RCCHECK(rmw_uros_sync_session(10));
  unsigned long long ros_time_ms = rmw_uros_epoch_millis();
  // now we can find the difference between ROS time and uC time
  time_offset = ros_time_ms - now;
}

struct timespec getTime()
{
  struct timespec tp = {0};
  // add time difference between uC time and ROS time to
  // synchronize time with ROS
  unsigned long long now = millis() + time_offset;
  tp.tv_sec = now / 1000;
  tp.tv_nsec = (now % 1000) * 1000000;

  return tp;
}

void rclErrorLoop()
{
  while (true)
  {
    flashLED(2);
  }
}

void flashLED(int n_times)
{
  for (int i = 0; i < n_times; i++)
  {
  }
  delay(1000);
}

void tcaSelect(uint8_t i)
{
  if (i > 7)
    return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}
