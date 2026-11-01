#ifndef __ROS_CMD_VEL_H__
#define __ROS_CMD_VEL_H__

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

extern rclc_executor_t executor;
extern rcl_node_t node;

void setupMicroROS();

#endif