#include "rosCmdVel.h"
#include "RobotController.h"
#include "rosEncoderDeltasGyroZ.h"
#include "lidar_publish.h"
#include "rosCmdVel.h"
#include "lidar_publish.h"
#include "analysis_one.h"
#include "secrets.h"

RobotController robot;

int motorControlPeriod_ms = 10; // 모터 속도 제어 주기(ms)
int motorControlPeriod_us = motorControlPeriod_ms*1000;

const unsigned long interval = motorControlPeriod_us;
unsigned long prev_us;

void setup() {
  Serial.begin(115200); 

  // Wi-Fi 재시도 루프
  while (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    WiFi.setAutoReconnect(true);
    WiFi.persistent(true);
    delay(2000);
    Serial.println("Wi-Fi 연결 재시도 중...");
  }

  // NTP 서버와 동기화 (UTC+9 = 한국 표준시)
  Serial.println("WiFi 연결 완료!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  Serial.print("DNS: ");
  Serial.println(WiFi.dnsIP());

  //여러 NTP 서버 지정
  configTime(0, 0, "216.239.35.8", "pool.ntp.org", "time.nist.gov");
  // configTime(9 * 3600, 0,
  //          "kr.pool.ntp.org",
  //          "time.google.com",
  //          "time.windows.com");

  Serial.println("NTP 동기화 시작");

  struct tm timeinfo;
  bool synced = false;

  for (int i = 0; i < 30; i++) {
      if (getLocalTime(&timeinfo, 1000)) {
          Serial.printf("NTP 성공: %s", asctime(&timeinfo));
          synced = true;
          break;
      }

      Serial.println("NTP 대기 중...");
      delay(500);
  }

  if (!synced) {
      Serial.println("NTP 동기화 실패");
  }

  setupMicroROS();
  encoder_deltas_gryoz_pub_init(node);
  init_lidar_publish(node);

  robot.begin();

  prev_us = micros();
}

unsigned long lastPrintTime = 0;
unsigned long whileTimeAccum = 0;  // while문 수행 누적 시간
unsigned long whileStart = 0;
unsigned long whileEnd = 0;
unsigned long whileTimeAccum2 = 0;
unsigned long whileStart2 = 0;
unsigned long whileEnd2 = 0;

uint8_t buf[BUF_SIZE];

int parse_cnt=0;
int pub_cnt=0;

struct timespec tv = {0, 0};
int32_t sec;
uint32_t nanosec;

void loop() {
  // while 문 시작 시간 기록
  whileStart = micros();

  // FIFO에 있는 바이트 수 확인
  size_t avail = LIDAR_SERIAL.available();
  while (avail > 0) {
    whileStart2 = micros();
    // 블록 단위로 읽을 수 있는 최대 바이트 수 계산
    size_t toRead = (avail > BUF_SIZE) ? BUF_SIZE : avail;

    // 한 번에 읽기
      size_t n = LIDAR_SERIAL.readBytes(buf, toRead);
      for(int i=0;i<n;i++) {
        uint8_t byte = buf[i];
        if (AnalysisOne(byte)) {
          
          if (parse_cnt==0) {
            // struct timeval tv;
            // gettimeofday(&tv, NULL);   // NTP 동기화된 시간 가져오기
            // sec  = tv.tv_sec;
            // nanosec = tv.tv_usec * 1000;
            clock_gettime(CLOCK_REALTIME, &tv);
            sec = tv.tv_sec;
            nanosec = tv.tv_nsec;
          }

          Parse();
          parse_cnt++;              
          
          if (parse_cnt == 56) { // =4000Hz/6Hz/12
            // 1. sec 복사 (4바이트)
            memcpy(&tx_buffer[0], &sec, sizeof(int32_t));
            // 2. nanosec 복사 (4바이트)
            memcpy(&tx_buffer[4], &nanosec, sizeof(uint32_t));

          for (int seg = 0; seg < SEGMENTS; seg++) {
            tx_buffer[8] = seg;  // segment index

            memcpy(&tx_buffer[9],
                  (uint8_t*)&ranges[seg * SEG_SIZE],
                  SEG_SIZE * sizeof(uint16_t));

            lidar_publish();
            pub_cnt++;
          }
          parse_cnt=0;
        } 
      }
    }

    // 남은 데이터 수 업데이트
    avail = LIDAR_SERIAL.available();
    whileEnd2 = micros();
    whileTimeAccum2 += (whileEnd2 - whileStart2);  
  }
 
  // while 문 끝난 시간
  whileEnd = micros();
  whileTimeAccum += (whileEnd - whileStart);

  static unsigned long motorControlTimeAccum = 0;
  static unsigned long motorControlCntAccum = 0;

  unsigned long curr_us = micros();
  if(curr_us - prev_us >= interval) {
    double dt = (curr_us - prev_us)/1000000.0;
    prev_us = curr_us; // 기준 시간 갱신

    static int loop_cnt=0;
    loop_cnt++;
    if (loop_cnt%50 == 0)
      Serial.printf("dt : %f\n", dt);

    unsigned long motorControlStart = micros();

    robot.updateMotorSpeedsRamped(dt);
    encoder_deltas_gryoz_publish();

    unsigned long motorControlEnd = micros();
    motorControlTimeAccum += (motorControlEnd - motorControlStart);
    motorControlCntAccum ++;
  }

  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));  

  // 1초마다 출력
  unsigned long currentTime = millis();
  if (currentTime - lastPrintTime >= 1000) {
    Serial.print("While time (us): ");
    Serial.print(whileTimeAccum);
    Serial.print(" | While2 time (us): ");
    Serial.print(whileTimeAccum2);
    Serial.print(" | publish count: ");
    Serial.print(pub_cnt);
    
    Serial.print(" | Motor Control time (us): ");
    Serial.print(motorControlTimeAccum);
    Serial.print(" | Motor Control count: ");
    Serial.print(motorControlCntAccum);

    Serial.print(" | total time (us): ");
    Serial.println(whileTimeAccum+motorControlTimeAccum);

    whileTimeAccum = 0;  // 초기화
    whileTimeAccum2 = 0;  // 초기화
    pub_cnt=0;

    motorControlTimeAccum = 0;  // 초기화
    motorControlCntAccum = 0; // 초기화

    lastPrintTime = currentTime;
    
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("⚠️ Wi-Fi disconnected, rebooting ESP32...");
      delay(100);
      ESP.restart();   // 🔁 재부팅 
    }

#define AGENT_PING_TIMEOUT 200   // ms
#define AGENT_PING_TRIES   5

    if (rmw_uros_ping_agent(AGENT_PING_TIMEOUT, AGENT_PING_TRIES) != RMW_RET_OK) {
      Serial.println("⚠️ micro-ROS agent disconnected, rebooting ESP32...");
      delay(100);
      ESP.restart();   // 🔁 재부팅
    } else {
      Serial.println("⚠️ micro-ROS agent is active!");
    }    
  }   
}

// 인코더 타임 스탬프 추가
// 라이다 타임 스탬프 추가
// 라이다 1회전 데이터를 4개로 나누어 보냄(3개->4개, 버퍼 크기 제한 때문에)
// 라이다 데이터 순서 조정(0,1,2,...,718,719(시계 방향 정렬) -> 0,719,718,...,2,1(반시계 방향 정렬))
// 인코더 데이터 std_msgs__msg__Int64MultiArray 타입 사용 - 시간 정보 보내기 위해
// Battery ADC 값, WiFi RSSI 값 인코더+자이로Z에 더해서 보냄

// 간단 테스트
// 1. ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v4
// 2. python3 my_teleop_twist_keyboard.py
// 3. rviz2 # Add>TF, Fixed Frame>odom
// 4. 키보드 조종

// _31_publish_odom_joint_states 자동차 업로드 테스트
// 1. ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro robot2.urdf.xacro)"
// 2. rviz2
// 3. python3 my_teleop_twist_keyboard.py
// 4. ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v4
// 5. 현재 소스 업로드
// 6. rviz2 시뮬레이션 시작