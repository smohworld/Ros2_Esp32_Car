#ifndef __ANALYSIS_ONE_H__
#define __ANALYSIS_ONE_H__

#include <micro_ros_arduino.h>
#include "lidar_frame.h"

extern LiDARFrameTypeDef datapkg_;
extern uint16_t ranges[SCAN_BUFFER_SIZE];
extern uint8_t tx_buffer[4 + 4 + 1 + SEG_SIZE * 2]; // sec, nanosec, index

bool AnalysisOne(uint8_t byte);
void Parse();

#endif