#include "rosCmdVel.h"
#include "RobotController.h"
#include "secrets.h"

static rcl_subscription_t twist_sub;
static geometry_msgs__msg__Twist twist_msg;

static rcl_allocator_t allocator;
static rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;

static void twist_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

  float v = msg->linear.x;   // 선속도
  float w = msg->angular.z;  // 각속도

  robot.setTargetSpeeds(v, w);
}

void setupMicroROS() {
  // Wi-Fi micro-ROS 연결
  set_microros_wifi_transports(WIFI_SSID, WIFI_PASSWORD, AGENT_IP, AGENT_PORT);
  delay(2000);

  allocator = rcl_get_default_allocator();

  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_twist_sub_node", "", &support);

  rclc_subscription_init_default(
    &twist_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"
  );
  
  rclc_executor_init(&executor, &support.context, 1, &allocator); // 1 - executor가 관리할 수 있는 핸들(콜백)의 개수
  rclc_executor_add_subscription(&executor, &twist_sub, &twist_msg, &twist_callback, ON_NEW_DATA);
}
