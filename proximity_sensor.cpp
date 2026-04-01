#include <Wire.h>
#include <stdint.h>
#include <cstdint>
#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_srvs/srv/trigger.h>

// Error handling macros
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// ROS 2 entities
rcl_publisher_t publisher_1;
rcl_publisher_t publisher_2;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
rcl_timer_t timer_closed;

// Init options
rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();

// Message types
std_msgs__msg__Float32MultiArray proximity_data;
std_msgs__msg__Float32MultiArray proximity_low_level_class;

// Service entities
rcl_service_t service;
std_srvs__srv__Trigger_Request req;
std_srvs__srv__Trigger_Response res;

// Counter
uint32_t cnt = 0;

// I2C address and register definitions
#define QT1070_ADDR       0x1B
#define REG_KEY0_SIG_MSB  4  // First key signal MSB start address
#define REG_CALIBRATE     56
#define REG_RESET         57

// Sensor configuration
#define NUM_SENSORS 2
#define NUM_KEYS    7
#define DELTA_THRESHOLD          50
#define BASELINE_COLLECTION_TIME 5000UL
#define IGNORE_THRESHOLD         1000
#define NOT_TOUCHED_TIME         0xFFFFFFFF
#define I2C_CLOCK_SPEED          400000  // 400kHz I2C clock speed
#define CACHE_UPDATE_INTERVAL    20      // 20ms cache update interval
#define CLOSED_THRESHOLD         300   // Threshold for considering a sensor "closed"

// Sensor data storage
static uint16_t baseline[NUM_SENSORS][NUM_KEYS];
static bool     baselineSet[NUM_SENSORS];
static uint32_t accumKeySignal[NUM_SENSORS][NUM_KEYS];
static uint32_t sampleCount[NUM_SENSORS];

static bool     wasTouched[NUM_SENSORS][NUM_KEYS];
static bool     isTouchedNow[NUM_SENSORS][NUM_KEYS];
static uint32_t touchStartTime[NUM_SENSORS][NUM_KEYS];
static float    filteredDelta[NUM_SENSORS][NUM_KEYS];

// Exponential filter coefficients
float alphaForKeySensor0[NUM_KEYS] = {0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8};
float alphaForKeySensor1[NUM_KEYS] = {0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8};

// Cached signal values
static uint16_t cachedSignals[NUM_SENSORS][NUM_KEYS];
static unsigned long lastCacheUpdateTime = 0;

// Function prototypes
void error_loop();
void timer_callback(rcl_timer_t * timer, int64_t last_call_time);
void closed_timer_callback(rcl_timer_t * timer, int64_t last_call_time);
void initSensor(uint8_t sensorIndex);
void resetSensor(uint8_t sensorIndex);
void calibrateSensor(uint8_t sensorIndex);
void readAllKeysAtOnce(uint8_t sensorIndex);
uint16_t getKeySignal(uint8_t sensorIndex, uint8_t keyIndex);

// Error handling function
void error_loop() {
  while(1) {
    delay(100);
  }
}

// Timer callback for publishing proximity data
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher_1, &proximity_data, NULL));
    cnt = 0;
  }
}

// Timer callback for publishing and resetting closed status
void closed_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // Publish proximity_low_level_class (2Hz)
    RCSOFTCHECK(rcl_publish(&publisher_2, &proximity_low_level_class, NULL));

    // Reset proximity_low_level_class
    for (size_t i = 0; i < proximity_low_level_class.data.size; i++) {
      proximity_low_level_class.data.data[i] = 0;
    }
  }
}

void setup() {
  // Use micro-ROS for communication
  set_microros_serial_transports(Serial);
  delay(100);
  
  allocator = rcl_get_default_allocator();

  // Initialize I2C with higher clock speed
  Wire.begin();
  Wire.setClock(I2C_CLOCK_SPEED);
  
  Wire1.begin();
  Wire1.setClock(I2C_CLOCK_SPEED);

  // Set ROS2 domain ID
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, 0));  
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

  // Create ROS2 node
  RCCHECK(rclc_node_init_default(&node, "proximity", "", &support));

  // Create publishers
  RCCHECK(rclc_publisher_init_default(
    &publisher_1,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "proximity_data"));

  RCCHECK(rclc_publisher_init_default(
    &publisher_2,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "proximity_low_level_class"));

  // Create timer for data publishing (200Hz)
  const unsigned int Nano_timer_timeout = 5000000; // 5ms = 200Hz
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    Nano_timer_timeout,
    timer_callback));

  // Create timer for closed state publishing (2Hz)
  const unsigned int Closed_timer_timeout = 250000000; // 500ms = 2Hz
  RCCHECK(rclc_timer_init_default(
    &timer_closed,
    &support,
    Closed_timer_timeout,
    closed_timer_callback));

  // Initialize executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_closed));

  // Initialize message data structures
  proximity_data.data.capacity = 10;
  proximity_data.data.size = 10;
  proximity_data.data.data = (float*) malloc(10 * sizeof(float));

  proximity_low_level_class.data.capacity = 10;
  proximity_low_level_class.data.size = 10;
  proximity_low_level_class.data.data = (float*) malloc(10 * sizeof(float));

  // Initialize sensors
  for (uint8_t s = 0; s < NUM_SENSORS; s++) {
    initSensor(s);
    baselineSet[s] = false;
    sampleCount[s] = 0;

    for (uint8_t k = 0; k < NUM_KEYS; k++) {
      accumKeySignal[s][k] = 0;
      wasTouched[s][k] = false;
      isTouchedNow[s][k] = false;
      touchStartTime[s][k] = NOT_TOUCHED_TIME;
      filteredDelta[s][k] = 0.0f;
      cachedSignals[s][k] = 0;
    }
  }
  
  // Initialize message arrays to zero
  for (size_t i = 0; i < proximity_data.data.size; i++) {
    proximity_data.data.data[i] = 0.0f;
    proximity_low_level_class.data.data[i] = 0.0f;
  }
}

void loop() {
  static unsigned long startTime = 0;
  static bool allBaselineDone = false;
  static unsigned long lastSampleTime = 0;
  unsigned long currentTime = millis();

  // Update I2C data cache
  if (currentTime - lastCacheUpdateTime >= CACHE_UPDATE_INTERVAL) {
    for (uint8_t s = 0; s < NUM_SENSORS; s++) {
      readAllKeysAtOnce(s);
    }
    lastCacheUpdateTime = currentTime;
  }

  // (1) Collect baseline data
  if (!allBaselineDone) {
    if (startTime == 0) startTime = currentTime;
    
    if (currentTime - lastSampleTime >= 50) { // 50ms sampling interval
      lastSampleTime = currentTime;

      for (uint8_t s = 0; s < NUM_SENSORS; s++) {
        if (!baselineSet[s]) {
          for (uint8_t k = 0; k < NUM_KEYS; k++) {
            accumKeySignal[s][k] += getKeySignal(s, k);
          }
          sampleCount[s]++;
        }
      }

      if (currentTime - startTime >= BASELINE_COLLECTION_TIME) {
        for (uint8_t s = 0; s < NUM_SENSORS; s++) {
          if (!baselineSet[s]) {
            for (uint8_t k = 0; k < NUM_KEYS; k++) {
              baseline[s][k] = (uint16_t)(accumKeySignal[s][k] / sampleCount[s]);
            }
            baselineSet[s] = true;
          }
        }
        allBaselineDone = baselineSet[0] && baselineSet[1];
      }
    }
    if (!allBaselineDone) return;
  }

  // (2) Calculate delta and determine touch state
  const uint8_t index_map[NUM_SENSORS][NUM_KEYS] = {
    {0 ,2, 1, 3, -1, -1, -1},  // Sensor 0 mapping
    {8, 9, 4, 6, 5, 7,-1}     // Sensor 1 mapping
  };
  
  for (uint8_t s = 0; s < NUM_SENSORS; s++) {
    for (uint8_t k = 0; k < NUM_KEYS; k++) {
      if (index_map[s][k] == -1) continue; // Skip unused keys
  
      uint16_t sig = getKeySignal(s, k);
      int32_t rawDelta = (int32_t)sig - (int32_t)baseline[s][k];
      
      // Filter out negative or extremely large values
      rawDelta = (rawDelta < 0 || rawDelta > IGNORE_THRESHOLD) ? 0 : rawDelta;
  
      // Apply exponential filter
      float alpha = (s == 0) ? alphaForKeySensor0[k] : alphaForKeySensor1[k];
      filteredDelta[s][k] = alpha * rawDelta + (1.0f - alpha) * filteredDelta[s][k];
  
      // Update proximity_data
      proximity_data.data.data[index_map[s][k]] = (float)filteredDelta[s][k];

      // Update proximity_low_level_class if value exceeds threshold
      if (proximity_data.data.data[index_map[s][k]] >= CLOSED_THRESHOLD) {
        proximity_low_level_class.data.data[index_map[s][k]] = 1;
      }
    }
  }

  // Execute ROS2 callbacks
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
}

// ─────────────────────────────────────
// 센서 초기화 함수 (reset + calibrate)
// ─────────────────────────────────────
void initSensor(uint8_t sensorIndex) {
  resetSensor(sensorIndex);
  delay(20);

  calibrateSensor(sensorIndex);
  delay(50);
  
  // Serial 디버깅 코드 제거
}

void resetSensor(uint8_t sensorIndex) {
  if(sensorIndex == 0) {
    Wire.beginTransmission(QT1070_ADDR);
    Wire.write(REG_RESET);
    Wire.write(0xFF);
    Wire.endTransmission();
  } else {
    Wire1.beginTransmission(QT1070_ADDR);
    Wire1.write(REG_RESET);
    Wire1.write(0xFF);
    Wire1.endTransmission();
  }
}

void calibrateSensor(uint8_t sensorIndex) {
  if(sensorIndex == 0) {
    Wire.beginTransmission(QT1070_ADDR);
    Wire.write(REG_CALIBRATE);
    Wire.write(0x01);
    Wire.endTransmission();
  } else {
    Wire1.beginTransmission(QT1070_ADDR);
    Wire1.write(REG_CALIBRATE);
    Wire1.write(0x01);
    Wire1.endTransmission();
  }
}

// ─────────────────────────────────────
// 모든 키 신호를 한 번에 읽어오기
// ─────────────────────────────────────
void readAllKeysAtOnce(uint8_t sensorIndex) {
  // 모든 키 데이터(14바이트)를 한 번에 요청
  uint8_t data[14];
  
  if(sensorIndex == 0) {
    Wire.beginTransmission(QT1070_ADDR);
    Wire.write(REG_KEY0_SIG_MSB);
    Wire.endTransmission(false);
    
    // 한 번의 requestFrom으로 모든 키 데이터 요청 (14바이트 = 7키 x 2바이트)
    if(Wire.requestFrom(QT1070_ADDR, (uint8_t)14) == 14) {
      for(uint8_t i = 0; i < 14; i++) {
        data[i] = Wire.read();
      }
      
      // 데이터 파싱 및 캐시 업데이트
      for(uint8_t k = 0; k < NUM_KEYS; k++) {
        cachedSignals[sensorIndex][k] = ((uint16_t)data[k*2] << 8) | data[k*2+1];
      }
    }
  } else {
    Wire1.beginTransmission(QT1070_ADDR);
    Wire1.write(REG_KEY0_SIG_MSB);
    Wire1.endTransmission(false);
    
    if(Wire1.requestFrom(QT1070_ADDR, (uint8_t)14) == 14) {
      for(uint8_t i = 0; i < 14; i++) {
        data[i] = Wire1.read();
      }
      
      for(uint8_t k = 0; k < NUM_KEYS; k++) {
        cachedSignals[sensorIndex][k] = ((uint16_t)data[k*2] << 8) | data[k*2+1];
      }
    }
  }
}

// ─────────────────────────────────────
// 캐시된 키 신호값 반환
// ─────────────────────────────────────
uint16_t getKeySignal(uint8_t sensorIndex, uint8_t keyIndex) {
  return cachedSignals[sensorIndex][keyIndex];
}