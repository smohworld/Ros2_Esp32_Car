#include "rosEncoderDeltasGyroZ.h"
#include "rosCmdVel.h"
#include "RobotController.h"

static rcl_publisher_t encoder_gyroz_pub;
static std_msgs__msg__Int64MultiArray encoder_gyroz_msg;

static int64_t encoder_gyroz_buf[MSG_SIZE+1+1] = {0,}; // + battery + rssi

void encoder_deltas_gryoz_pub_init(rcl_node_t& node) {
  encoder_gyroz_msg.data.data = encoder_gyroz_buf;
  encoder_gyroz_msg.data.size = MSG_SIZE+1+1;
  encoder_gyroz_msg.data.capacity = MSG_SIZE+1+1;

  rclc_publisher_init_best_effort(
    &encoder_gyroz_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64MultiArray),
    "encoder_deltas_gyroz_bat_rssi"
  );
}

void encoder_deltas_gryoz_publish() {
  int32_t sec;
  uint32_t nanosec;
  robot.getEncoderStamp(sec, nanosec);

  long dEncPosFL, dEncPosRL, dEncPosFR, dEncPosRR;
  robot.getDeltaEncoderPos(dEncPosFL, dEncPosRL, dEncPosFR, dEncPosRR);
  int16_t GyZCalib = robot.getGyroZCalib();
  int32_t batVoltage = robot.getBatteryVoltage();
  long rssi_dbm = WiFi.RSSI(); // RSSI 읽기 (dBm 단위)
  // 안전한 범위로 클램핑 (ESP32 내부에서 -127~0이지만, 혹시 모를 이상값 대비)
  if (rssi_dbm > 127)  rssi_dbm = 127;
  if (rssi_dbm < -128) rssi_dbm = -128;

  encoder_gyroz_buf[0] = sec;
  encoder_gyroz_buf[1] = nanosec;
  encoder_gyroz_buf[2] = dEncPosFL;
  encoder_gyroz_buf[3] = dEncPosRL;
  encoder_gyroz_buf[4] = dEncPosFR;
  encoder_gyroz_buf[5] = dEncPosRR;
  encoder_gyroz_buf[6] = GyZCalib;

  encoder_gyroz_buf[7] = batVoltage;
  encoder_gyroz_buf[8] = rssi_dbm;

  static int loop_cnt=0;
  loop_cnt++;
  if (loop_cnt % 50 == 0)
    Serial.printf("%6d:%6d,%6d,%6d,%6d,%6d,%6d,%6d,%6d\n", sec, nanosec,
      dEncPosFL, dEncPosRL, dEncPosFR, dEncPosRR, GyZCalib, rssi_dbm);

  rcl_publish(&encoder_gyroz_pub, &encoder_gyroz_msg, NULL);
}