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
