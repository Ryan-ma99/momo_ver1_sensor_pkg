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
