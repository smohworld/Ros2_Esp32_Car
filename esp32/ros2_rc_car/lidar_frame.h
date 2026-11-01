#ifndef __LIDAR_FRAME_H__
#define __LIDAR_FRAME_H__

enum {
  PKG_HEADER = 0x54,
  PKG_VER_LEN = 0x2C,
  POINT_PER_PACK = 12,
};

typedef struct __attribute__((packed)) {
  uint16_t distance;
  uint8_t intensity;
} LidarPointStructDef;

typedef struct __attribute__((packed)) {
  uint8_t header;
  uint8_t ver_len;
  uint16_t speed;
  uint16_t start_angle;
  LidarPointStructDef point[POINT_PER_PACK];
  uint16_t end_angle;
  uint16_t timestamp;
  uint8_t crc8;
} LiDARFrameTypeDef;

#define SCAN_BUFFER_SIZE 720 //667 // = 4000Hz/6Hz
#define SEGMENTS 4
#define SEG_SIZE (SCAN_BUFFER_SIZE / SEGMENTS)
//LD14P 라이다 사양
//거리 측정 범위: 0.15 - 8m
//스캐닝 주파수: 6Hz
//거리 측정 정확도: 1°
//거리 측정 주파수: 4000Hz
//스캐닝 각도: 360°

#endif
