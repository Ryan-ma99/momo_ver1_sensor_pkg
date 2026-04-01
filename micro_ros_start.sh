#!/bin/bash

# 1. 실행할 장치 리스트
DEVICES=(
  "right_arm_1_teensy"
  "right_arm_2_teensy"
  "left_arm_1_teensy"
  "left_arm_2_teensy"
  "chest_teensy"
  "proximity_teensy"
)

# 2. 종료 함수 (프로세스 그룹 전체 사살)
cleanup() {
  echo -e "\n\n🛑 [EMERGENCY STOP] 모든 micro-ROS 에이전트를 강제 종료합니다..."
  
  # 현재 스크립트의 프로세스 그룹 ID(PGID)를 찾아 그 그룹에 속한 모든 놈들을 한꺼번에 kill
  # 이 방식이 jobs -p 보다 훨씬 강력합니다.
  sudo pkill -9 -f "micro_ros_agent" 2>/dev/null
  
  # 혹시나 남아있을지 모르는 자식 프로세스들 정리
  trap - SIGINT SIGTERM # 무한 루프 방지
  kill -- -$$ 2>/dev/null
  
  echo "✅ 정리 완료. 터미널이 깨끗해졌습니다."
  exit 0
}

# Ctrl+C 뿐만 아니라 스크립트가 꺼지는 모든 상황에 대비
trap cleanup SIGINT SIGTERM EXIT

echo "🚀 micro-ROS 에이전트 6개를 동시 실행합니다..."

for dev_name in "${DEVICES[@]}"; do
  port="/dev/$dev_name"
  echo "▶️  실행 중: $port"
  
  # 각 에이전트를 백그라운드에서 실행
  # 팁: 로그가 너무 시끄러우면 뒤에 '> /dev/null 2>&1'를 붙여보세요.
  ros2 run micro_ros_agent micro_ros_agent serial --dev "$port" & 
done

echo "------------------------------------------------------"
echo "✅ 모든 에이전트가 백그라운드에서 대기 중입니다."
echo "⏳ [Ctrl+C]를 누르면 모든 프로세스를 즉시 박멸합니다."
echo "------------------------------------------------------"

# 백그라운드 프로세스가 살아있는 동안 무한 대기
while true; do
    sleep 1
done
