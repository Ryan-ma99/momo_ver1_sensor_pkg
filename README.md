# momo_ver1_sensor_pkg
이 패키지는 momo_ver1의 센서데이터를 ros2 topic 형태로 받을 수 있게 teensy를 프로그래밍 할 수 있는 패키지 입니다.  
platform_io 패키지이므로 VScode platform io extension을 설치하여 이용

1. tytools 설치

  tytools은 다수의 teensy 보드를 관리할 수 있는 tool  
```
sudo mkdir -p /etc/apt/keyrings
sudo curl -fsSL https://download.koromix.dev/debian/koromix-archive-keyring.gpg -o /etc/apt/keyrings/koromix-archive-keyring.gpg
echo "deb [signed-by=/etc/apt/keyrings/koromix-archive-keyring.gpg] https://download.koromix.dev/debian stable main" | sudo tee /etc/apt/sources.list.d/koromix.dev-stable.list > /dev/null
sudo apt update
sudo apt install tytools
```

2. teensy.rule 파일 설정
teensy의 시리얼 번호 확인
```
tycmd list
```
앞 숫자가 teensy 고유 시리얼 번호
```
#결과
add 14808190-Teensy Teensy 4.0 (USB Serial)
add 16846430-Teensy Teensy 4.0 (USB Serial)
```
teensy.rule 파일 작성
```
sudo gedit /etc/udev/rules.d/99-teensy.rules 
```
생성된 파일 안에 아래 코드 작성
```
SUBSYSTEM=="tty", ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="0483", ATTRS{serial}=="16845300", SYMLINK+="right_arm_1_teensy"
SUBSYSTEM=="tty", ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="0483", ATTRS{serial}=="16956780", SYMLINK+="right_arm_2_teensy"
SUBSYSTEM=="tty", ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="0483", ATTRS{serial}=="16846430", SYMLINK+="left_arm_1_teensy"
SUBSYSTEM=="tty", ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="0483", ATTRS{serial}=="14808190", SYMLINK+="left_arm_2_teensy"
SUBSYSTEM=="tty", ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="0483", ATTRS{serial}=="02330020", SYMLINK+="chest_teensy"
SUBSYSTEM=="tty", ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="0483", ATTRS{serial}=="00000011", SYMLINK+="proximity"
```
rule 적용 후 usb 해제 후 다시 연결
```
sudo udevadm control --reload-rules
sudo udevadm trigger
```
3.VScode로 업로드  
본 패키지는 src/main.cpp에 있는 코드를 해당 teensy에 업로드하는 패키지  
pneumatic_sensor.cpp와 proximity_sensor.cpp의 코드를 src/main.cpp에 붙여넣어 업로드할 코드 설정  
아래 사진의 pick a folder를 눌러 해당 패키지 open

<img width="375" height="426" alt="Screenshot from 2026-04-01 14-28-33" src="https://github.com/user-attachments/assets/942fa564-cfcd-41bc-b90a-9d004af5e26a" />

왼쪽 상당 upload할 teensy를 골라 해당 teensy에 코드 upload  
<img width="415" height="729" alt="Screenshot from 2026-04-01 14-30-58" src="https://github.com/user-attachments/assets/1b7611e2-b0ae-4a08-8b09-36ea669443c6" />

4.micro_ros agent 실행  
micro_ros 설치가 완료된 상태라면 패키지 내 micro_ros_start.sh 실행 
```
./micro_ros_start.sh
```
ctrl+c 로 종료

5. code 설명
pneumatic_sensor.cpp

공압패드 데이터의 index는 아래 부분에서 설정 (현재 index는 패키지 pad_index.jpg에서 확인)
```
// 15개 채널 사용
// 원하는 MUX 채널 순서 (15개)
int mux_order[15] = {8,10,9, 5,3,2,4,  6, 0, 1, 7,  13,12,14,11  };
```
```
#left_link1_data
int mux_order[15] = {9, 10, 8,  4, 2, 3, 5,  6, 0, 1, 7,  13,12,14,11  };
#left_link2_data
int mux_order[15] = {7,0,1,  3, 6, 5, 2,  4, 8, 9, 10,  14,11,12,13  };
#right_link1_data
int mux_order[15] = {8,10,9, 5,3,2,4,  6, 0, 1, 7,  13,12,14,11  };
#right_link2_data
int mux_order[15] = {7,0,1,  3, 6, 5, 2,  4, 8, 9, 10,  14,11,12,13  };
#chest_data
int mux_order[16] = {5, 7, 4, 6, 12, 3, 8, 10, 11, 9, 0, 1, 2, 13, 14, 15};
```
