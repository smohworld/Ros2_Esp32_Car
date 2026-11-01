#ifndef __LIDAR_PUBLISH_H__
#define __LIDAR_PUBLISH_H__

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/u_int8_multi_array.h>

#include "lidar_frame.h"

#define LIDAR_SERIAL Serial1
#define LIDAR_BAUD 230400
#define UART_RX 16 // 35 (makerspet)
#define UART_TX 17 // 27 (makerspet)

#define BUF_SIZE 64

void init_lidar_publish(rcl_node_t& node);
void lidar_publish();

#endif