#ifndef __ROS_ENCODER_DELTAS_GYROZ_H__
#define __ROS_ENCODER_DELTAS_GYROZ_H__

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int64_multi_array.h>

#define ENCODER_GYROZ_COUNT 7
#define MSG_SIZE (ENCODER_GYROZ_COUNT * 1) // 5개 × 2바이트 = 10바이트

void encoder_deltas_gryoz_pub_init(rcl_node_t& node);
void encoder_deltas_gryoz_publish();

#endif