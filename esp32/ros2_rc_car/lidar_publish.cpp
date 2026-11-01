#include "lidar_publish.h"
#include "analysis_one.h"

static rcl_publisher_t segment_pub;
static std_msgs__msg__UInt8MultiArray segment_msg;

void init_lidar_publish(rcl_node_t& node) {
  rclc_publisher_init_best_effort(
    &segment_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8MultiArray),
    "range_segment"
  );
  
  // raw 배열 데이터는 488바이트 이상에서 문제가 생김
  // 여기서는 4개의 세그먼트로 나누어 보냄
  // 720/4=180, 180*2+4+4+1=369바이트 단위로 보냄
  segment_msg.data.data = tx_buffer;
  segment_msg.data.size = 4 + 4 + 1 + SEG_SIZE * 2;
  segment_msg.data.capacity = 4 + 4 + 1 + SEG_SIZE * 2;

  LIDAR_SERIAL.begin(LIDAR_BAUD, SERIAL_8N1, UART_RX, UART_TX, false, 4096, 512); // 버퍼 크기 중요
}

void lidar_publish() {
  rcl_publish(&segment_pub, &segment_msg, NULL);
}