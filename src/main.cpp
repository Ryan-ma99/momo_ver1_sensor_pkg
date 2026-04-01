#include <stdint.h>
#include <cstdint>
#include <stdio.h>
#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <CD74HC4067.h>
#include <std_srvs/srv/trigger.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Publishers
rcl_publisher_t publisher_lpf;
rcl_publisher_t publisher_hpf;
rcl_publisher_t publisher_low_class;  // pneumatic_low_class 토픽용 publisher

// Subscription for offset reset
rcl_subscription_t subscriber;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

rcl_timer_t timer;           // 기존 타이머 (pressure 데이터용)
rcl_timer_t timer_low_class; // pneumatic_low_class 토픽용 타이머 (4Hz)

rcl_timer_callback_t callback;
std_msgs__msg__Bool sub_msg;
rcl_ret_t rc;
rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();

// Messages
std_msgs__msg__Float32MultiArray Pressure_msgs_lpf;
std_msgs__msg__Float32MultiArray Pressure_msgs_hpf;
std_msgs__msg__Float32MultiArray pneumatic_low_class_msgs;  // 새로운 토픽 메시지

// Service
rcl_service_t service;
std_srvs__srv__Trigger_Request req;
std_srvs__srv__Trigger_Response res;

CD74HC4067 my_mux(22, 23, 1, 2); // mux digital pin 정의

const int Analogpin = A7;
uint32_t cnt = 0;

bool prev_reset_state = false; // 이전 리셋 상태를 저장하는 변수

// 15개 채널 사용
// 원하는 MUX 채널 순서 (15개)
int mux_order[15] = {8,10,9, 5,3,2,4,  6, 0, 1, 7,  13,12,14,11  };
// 각 토픽 인덱스별 가중치 배열 - 토픽 인덱스 순서로 정의
float topic_weights[15] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

// 먹스 채널에 해당하는 가중치 값 (mux_order와 동일한 순서)
float mux_weights[15] = {0};

// 샘플링 시간
float dt = 0.0005f;  // 샘플링 주기 (0.5ms = 0.0005s)

// 1. 초기 노이즈 제거용 IIR LPF 필터 변수
float a = 0.125f;  // 현재값의 비중
float b = (1-a);  // 이전값의 비중
float prev_noise_lpf[15] = {0};

// 2. 오프셋 드리프트 제거용 HPF 변수
float fc_drift_hpf = 0.1f;  // 드리프트 제거용 매우 낮은 컷오프 주파수 (0.1Hz)
float drift_hpf_alpha = exp(-2.0f * PI * fc_drift_hpf * dt);
float prev_input_drift_hpf[15] = {0};
float prev_output_drift_hpf[15] = {0};
float stable_counter[15] = {0};
float stable_counter_[15] = {0};

// 3. 분석용 LPF 변수
float fc_lpf = 5.0f;  // 컷오프 주파수 (5Hz)
float lpf_alpha = 1.0f - exp(-2.0f * PI * fc_lpf * dt);
float prev_output_lpf[15] = {0};

// 4. 분석용 HPF 변수
float fc_hpf = 100.0f;  // 컷오프 주파수 (80Hz)
float hpf_alpha = exp(-2.0f * PI * fc_hpf * dt);
float prev_input_hpf[15] = {0};
float prev_output_hpf[15] = {0};

// loop() 함수에서 사용하는 추가 필터 변수들
float fc_lpf2 = 15.0f;  // 컷오프 주파수 (15Hz)
float fc_drift_hpf2 = 0.5f;  // 컷오프 주파수 (0.5Hz)
float lpf2_alpha = 1.0f - exp(-2.0f * PI * fc_lpf2 * dt);
static float prev_output_lpf2[15] = {0};
static float prev_output_drift_hpf2[15] = {0};
static float prev_input_drift_hpf2[15] = {0};
float drift_hpf2_alpha = exp(-2.0f * PI * fc_drift_hpf2 * dt);

// 오프셋 및 데드존 변수
float OffsetValue[15] = {0};
const float DEADZONE = 10.0f;

// pneumatic_low_class 관련 변수
float low_class_matrix[15][2] = {0};  // 15x2 매트릭스 (모두 0으로 초기화)
int hpf_threshold_count[15] = {0};    // HPF 임계값 초과 카운트
bool prev_hpf_over_threshold[15] = {false};  // 이전 HPF 값이 임계값 초과 여부

// float 소수점 출력 필터 함수
float roundToDecimal(float value, int decimal) {
    float multiplier = powf(10.0f, decimal);  // 10의 decimal승 계산
    return roundf(value * multiplier) / multiplier;  // 반올림 후 원래 자릿수로 복원
}

// 1. 노이즈 제거용 초기 LPF 적용 함수
float apply_noise_lpf(float input, int channel) {
  float a_adj = a;
  float b_adj = b;
  if (channel > 10) {
    a_adj = 0.15f;
    b_adj = 0.85f;
  }
  float output = a_adj * input + b_adj * prev_noise_lpf[channel];
  prev_noise_lpf[channel] = output;
  return output;
}

// 2. 오프셋 드리프트 제거용 HPF 함수
float apply_drift_hpf(float input, int channel) {
    float output = drift_hpf_alpha * (prev_output_drift_hpf[channel] + input - prev_input_drift_hpf[channel]);
    prev_input_drift_hpf[channel] = input;
    prev_output_drift_hpf[channel] = output;
    return output;
}

// 3. 분석용 LPF 함수
float apply_analysis_lpf(float input, int channel) {
    float output = lpf_alpha * input + (1.0f - lpf_alpha) * prev_output_lpf[channel];
    prev_output_lpf[channel] = output;
    return output;
}

// 4. 분석용 HPF 함수
float apply_analysis_hpf(float input, int channel) {
    float output = hpf_alpha * (prev_output_hpf[channel] + input - prev_input_hpf[channel]);
    prev_input_hpf[channel] = input;
    prev_output_hpf[channel] = output;
    return output;
}

// 오프셋 초기화 함수
void reset_all_offsets() {
  for (int i = 0; i < 15; i++) {
    float sum = 0;
    my_mux.channel(mux_order[i]);
    // 20번 측정하여 평균 계산
    for (int j = 0; j < 20; j++) {
      sum += analogRead(Analogpin);
      delayMicroseconds(100);
    }
    OffsetValue[i] = sum / 20.0f;
    OffsetValue[i] = OffsetValue[i] * mux_weights[i];
    
    // 모든 필터 초기화
    prev_noise_lpf[i] = OffsetValue[i];
    prev_input_drift_hpf[i] = OffsetValue[i];
    prev_output_drift_hpf[i] = 0;
    prev_output_lpf[i] = 0;
    prev_input_hpf[i] = 0;
    prev_output_hpf[i] = 0;
    
    // 안정화 카운터 초기화
    stable_counter[i] = 0;
    stable_counter_[i] = 0;
    
    // 추가 필터 초기화
    prev_output_lpf2[i] = 0;
    prev_output_drift_hpf2[i] = 0;
    prev_input_drift_hpf2[i] = 0;
    
    // low_class 관련 초기화
    for (int j = 0; j < 2; j++) {
      low_class_matrix[i][j] = 0;
    }
    hpf_threshold_count[i] = 0;
    prev_hpf_over_threshold[i] = false;
  }
  delay(500);
}

void error_loop() {
  while(1) {
    delay(100);
  }
}

// 기존 타이머 콜백 (pressure 데이터용, 200Hz)
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher_lpf, &Pressure_msgs_lpf, NULL));
    RCSOFTCHECK(rcl_publish(&publisher_hpf, &Pressure_msgs_hpf, NULL));
    cnt = 0;
  }
}

// pneumatic_low_class 토픽용 타이머 콜백 (4Hz)
void timer_low_class_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // 매트릭스 데이터를 1차원 배열로 변환하여 메시지에 저장
    for (int i = 0; i < 15; i++) {
      for (int j = 0; j < 2; j++) {
        pneumatic_low_class_msgs.data.data[i * 2 + j] = low_class_matrix[i][j];
      }
    }
    RCSOFTCHECK(rcl_publish(&publisher_low_class, &pneumatic_low_class_msgs, NULL));
    
    // 발행 후 카운트 초기화 (HPF 임계값 초과 카운트)
    for (int i = 0; i < 15; i++) {
      hpf_threshold_count[i] = 0;
      // 매트릭스 값 초기화
      for (int j = 0; j < 2; j++) {
        low_class_matrix[i][j] = 0.0f;
      }
    }
  }
}

// 구독 콜백 함수 구현 (Bool 타입으로 오프셋 리셋)
void subscription_callback(const void * msgin) {
    const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
    
    // false에서 true로 변경되면 오프셋 초기화 실행
    if (msg->data && !prev_reset_state) {
        reset_all_offsets();
    }
    
    // 현재 상태 저장
    prev_reset_state = msg->data;
}

// 서비스 콜백 함수 (Trigger 타입으로 오프셋 리셋)
void service_callback(const void * req, void * res) {
  std_srvs__srv__Trigger_Response * res_in = 
    (std_srvs__srv__Trigger_Response *) res;
  
  // 오프셋 리셋 작업 수행
  reset_all_offsets();
  
  // Offset success message
  res_in->success = true;
  const char * message = "Reset Success";
  if (res_in->message.capacity == 0) {
    res_in->message.data = (char*)malloc(100);
    res_in->message.capacity = 100;
  }
  snprintf(res_in->message.data, res_in->message.capacity, "%s", message);
  res_in->message.size = strlen(res_in->message.data);
}

void setup() {
  Serial.begin(4000000);
  set_microros_serial_transports(Serial);
  digitalWrite(0, LOW);
  delay(2000);
  analogReadResolution(12);
  allocator = rcl_get_default_allocator();
  
  // mux_weights 초기화 - 토픽 가중치를 mux_order에 맞게 설정
  for (int i = 0; i < 15; i++) {
    mux_weights[i] = topic_weights[i]; // 인덱스 i에 해당하는 가중치 사용
  }

  // ROS2 도메인 ID 설정
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, 0));  
  
  // create init_options init옵션 정의 
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

  // create node 노드 이름 정의 
  RCCHECK(rclc_node_init_default(&node, "right_link1_gauge", "", &support));

  // create publishers
  RCCHECK(rclc_publisher_init_default(
    &publisher_lpf,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "right_link1_data"));

  RCCHECK(rclc_publisher_init_default(
    &publisher_hpf,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "right_link1_data_hpf"));

  RCCHECK(rclc_publisher_init_default(
    &publisher_low_class,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "right_link1_pneumatic_low_level_class"));

  // create subscription (Bool 타입으로 오프셋 리셋)
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "offset_reset"));

  // create service (Trigger 타입으로 오프셋 리셋)
  RCCHECK(rclc_service_init_default(&service, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger), "OffsetReset"));

  // create timer (기존 타이머, 200Hz)
  const unsigned int Nano_timer_timeout = 5000000;  // 5ms = 200Hz
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    Nano_timer_timeout,
    timer_callback));

  // create timer for low_class (4 Hz)
  const unsigned int Low_class_timer_timeout = 250000000;  // 250ms = 4Hz
  RCCHECK(rclc_timer_init_default(
    &timer_low_class,
    &support,
    Low_class_timer_timeout,
    timer_low_class_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 8, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_low_class));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &sub_msg, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_service(&executor, &service, &req, &res, service_callback));

  // Initialize LPF message (15개)
  Pressure_msgs_lpf.data.capacity = 15;
  Pressure_msgs_lpf.data.size = 15;
  Pressure_msgs_lpf.data.data = (float*) malloc(15 * sizeof(float));

  // Initialize HPF message (15개)
  Pressure_msgs_hpf.data.capacity = 15;
  Pressure_msgs_hpf.data.size = 15;
  Pressure_msgs_hpf.data.data = (float*) malloc(15 * sizeof(float));
  
  // pneumatic_low_class 메시지 초기화 (15x2 = 30)
  pneumatic_low_class_msgs.data.capacity = 30;
  pneumatic_low_class_msgs.data.size = 30;
  pneumatic_low_class_msgs.data.data = (float*) malloc(30 * sizeof(float));
  
  // 초기화 (모든 값을 0으로)
  for (int i = 0; i < 30; i++) {
    pneumatic_low_class_msgs.data.data[i] = 0.0f;
  }
  
  // 오프셋 값 계산 및 필터 초기화
  for (int i = 0; i < 15; i++) {
    float sum = 0;
    // mux_order 배열에 정의된 순서대로 채널 선택
    my_mux.channel(mux_order[i]);
    
    // 여러 번 측정하여 평균 계산
    for (int j = 0; j < 20; j++) {
      sum += analogRead(Analogpin);
      delayMicroseconds(100);
    }
    OffsetValue[i] = sum / 20.0f;
    OffsetValue[i] = OffsetValue[i] * mux_weights[i];
    
    // 모든 필터 초기화
    prev_noise_lpf[i] = OffsetValue[i];
    prev_input_drift_hpf[i] = OffsetValue[i];
    prev_output_drift_hpf[i] = 0;
    prev_output_lpf[i] = 0;
    prev_input_hpf[i] = 0;
    prev_output_hpf[i] = 0;
  }
  
  // low_class_matrix 초기화
  for (int i = 0; i < 15; i++) {
    for (int j = 0; j < 2; j++) {
      low_class_matrix[i][j] = 0;
    }
    hpf_threshold_count[i] = 0;
    prev_hpf_over_threshold[i] = false;
  }
  
  delayMicroseconds(500);
}

void loop() {
  // 임시 저장 배열 (먹스 순서대로 처리한 결과를 저장)
  float temp_lpf_data[15] = {0};
  float temp_hpf_data[15] = {0};
  float temp2_hpf_data[15] = {0};
  float temp3_hpf_data[15] = {0};

  // 1. 모든 채널을 읽고 고급 필터링 처리
  for (int i = 0; i < 15; i++) {
    // mux_order 배열에 정의된 순서대로 채널 선택
    my_mux.channel(mux_order[i]);
    float currentValue = analogRead(Analogpin);
    
    // 1. 노이즈 제거용 1st order IIR LPF
    float noise_filtered = apply_noise_lpf(currentValue, i);
    noise_filtered = noise_filtered * mux_weights[i];

    // 2. 분석용 HPF 적용 (impulse filtering)
    float hpf_value = apply_analysis_hpf(noise_filtered, i);

    // 3. 분석용 LPF 적용 (indent filtering)
    float lpf_value = apply_analysis_lpf(noise_filtered, i);

    // 4. 동적 오프셋 조정 - for negative data
    if ((noise_filtered - OffsetValue[i]) < 0) {
      stable_counter_[i]++;
      if (stable_counter_[i] > 25) {
          OffsetValue[i] = OffsetValue[i] * 0.99f + noise_filtered * 0.01f;
          stable_counter_[i] = 0;
      }
    } else {
        stable_counter_[i] = 0;
    }

    // 5. 동적 오프셋 조정 - for under threshold
    if ((noise_filtered - OffsetValue[i]) <= 20.0f && (noise_filtered - OffsetValue[i]) > 0) {
      stable_counter[i]++;
      if (stable_counter[i] > 50) {
          OffsetValue[i] = OffsetValue[i] * 0.99f + noise_filtered * 0.01f;
          stable_counter[i] = 0;
      }
    } else {
        stable_counter[i] = 0;
    }

    // 6. 오프셋 제거
    lpf_value = lpf_value - OffsetValue[i];

    // 7. 음수값 처리
    if (lpf_value < 0) {
      lpf_value = 0;
    }
    
    // 8. 데드존 적용
    if (lpf_value < DEADZONE) {
        lpf_value = 0;
    } else {
        lpf_value = lpf_value - DEADZONE;
    }

    // 9. 압력값 스케일링 (ADC to pressure unit) 
    float pressure_psi = lpf_value / 4096.0f / 0.8f * 5.0f * 16.0f;
    temp_lpf_data[i] = roundToDecimal(pressure_psi, 2);
    
    // 10. HPF 절댓값 처리
    temp_hpf_data[i] = abs(hpf_value);

    // 11. 추가 LPF 필터링 (중복 필터링)
    temp2_hpf_data[i] = lpf2_alpha * temp_hpf_data[i] + (1.0f - lpf2_alpha) * prev_output_lpf2[i];
    prev_output_lpf2[i] = temp2_hpf_data[i];

    // 12. 조건부 드리프트 HPF
    if (temp2_hpf_data[i] < 6) {
      temp3_hpf_data[i] = drift_hpf2_alpha * (prev_output_drift_hpf2[i] + temp2_hpf_data[i] - prev_input_drift_hpf2[i]);
      prev_input_drift_hpf2[i] = temp2_hpf_data[i];
      prev_output_drift_hpf2[i] = temp3_hpf_data[i];
    } else {
      temp3_hpf_data[i] = temp2_hpf_data[i];
    }
    
    // 13. HPF 압력값 스케일링 및 증폭
    float hpf_pressure_psi = temp3_hpf_data[i] / 4096.0f / 0.8f * 3.3f * 16.0f;
    float amplification = 5.0f;  // 증폭 계수
    float amplified_HPF_Pout = hpf_pressure_psi * amplification;
    
    // 14. pneumatic_low_class 매트릭스 업데이트
    // [0] - pressure 값이 1 이상이면 1로, 3 이상이면 2로 설정
    if (pressure_psi >= 5.0f) {
      low_class_matrix[i][0] = 1.0f;
    }
    if (pressure_psi >= 10.0f) {
      low_class_matrix[i][0] = 2.0f;
    }
    
    // HPF 값이 임계값을 넘었는지 확인
    bool current_hpf_over_threshold = (amplified_HPF_Pout >= 0.1f);
    
    // [1] - HPF_Pressure 값이 1.0 이상이고 동시에 LPF 값이 5 이하일 때만 1로 설정
    if (current_hpf_over_threshold && pressure_psi <= 1.0f) {
      low_class_matrix[i][1] = 1.0f;
      
      // 상승 에지 검출 (0->1 전이)
      if (!prev_hpf_over_threshold[i]) {
        // HPF 임계값 초과 카운트 증가 (상승 에지에서만)
        hpf_threshold_count[i]++;
      }
    }
    
    // 현재 상태를 이전 상태로 저장
    prev_hpf_over_threshold[i] = current_hpf_over_threshold;
  }

  // 2. 처리 완료 후 결과 저장
  for (int i = 0; i < 15; i++) {
      Pressure_msgs_lpf.data.data[i] = temp_lpf_data[i];
      Pressure_msgs_hpf.data.data[i] = roundToDecimal(temp3_hpf_data[i] / 4096.0f / 0.8f * 3.3f * 16.0f * 5.0f, 2);
  }

  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
}