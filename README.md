# ESP-VTX
Digital,low latency video transmitter base on ESP32
origin project:https://github.com/jeanlemotan/esp32-cam-fpv

## Features:
It uses FEC encoding (2/3 currently configured) with 1400 byte packets and it achieves quite good performance:
* Up to 54Mbps video rate.
* More than 80 FPS at 320x240 or lower with 20-50 ms latency.
* 30+ FPS at 800x600 or 640x480 with 50-80 ms latency.
* 12 FPS at 1024x768 with >100 ms latency.
* run at any ESP-cam boards(like AI Thinker etc.) with OV2640.

## Theory
The data is received from the camera module as JPEG ar 20MHz I2S clock and passed directly to the wifi module and written to the SD card if the DVR is enabled.
The ESP camera component was modified to send the data as it's received from the DMA instead of frame-by-frame basis. This decreases latency quite significantly (10-20 ms) and reduces the need to allocate full frames in PSRAM.

The wifi data is send using packet injection with configurable rate - from 2 MB CCK to 72MB MCS7 - and power.

The air unit can also record the video straight from the camera to a sd card. The format is a rudimentary MJPEG without any header so when playing back the FPS will be whatever your player will decide.\
There is significant buffering when writing to SD (3MB at the moment) to work around the very regular slowdowns of sd cards.


The receiver is a Raspberry PI 4 with 2 wifi adapters in monitor mode (TL-WN722N). The adapters work as diversity receivers and the data is reconstructed from the FEC packets.

The JPEG decoding is done with turbojpeg to lower latency and - based on the resolution - can take between 1 and 7 milliseconds.\
It's then uploaded to 3 separate textures as YUV and converted to RGB in a shader.

The link is bi-directional so the ground station can send data to the air unit. At the moment it sends camera and wifi configuration data but I plan to have a full bi-directional serial port for telemetry coming soon.\
The back link uses 64byte packets with a 2/6 FEC encoding (so quite solid) at a low wifi rate (I think 1Mb).

This is very WIP at the moment, but it proves that the ESP32 can definitely stream video with very low latency. \
I plan to use this is a long range micro quad.

Here is a video shot at 30 FPS at 800x600 (video converted from the source mjpeg):

https://user-images.githubusercontent.com/10252034/116135308-43c08c00-a6d1-11eb-99b9-9dbb106b6489.mp4

## Building
### esp32 firmware:
- Uses esp-idf-v4.3-beta1, can probably work with newer idf versions. It needs to be properly installed (follow the instructions in the IDF documentation)
- Only the Ai Thinker esp cam board tested
- In the air_firmware, execute this: `idf.py -p /dev/tty.usbserial1 flash monitor`. Replace `tty.usbserial1` with your serial port.
- Make sure you place the board in flashing mode bu connecting IO0 to GND and resetting the board.
- After compiling and resetting, you should get some stats per second, smth like this in the console:\
`WLAN S: 695196, R: 350, E: 0, D: 0, % : 0 || FPS: 64, D: 401268 || D: 0, E: 0`\
`WLAN S: 761616, R: 420, E: 0, D: 0, % : 0 || FPS: 69, D: 443679 || D: 0, E: 0`\
`WLAN S: 763092, R: 420, E: 0, D: 0, % : 0 || FPS: 69, D: 449410 || D: 0, E: 0`\
`WLAN S: 764568, R: 420, E: 0, D: 0, % : 0 || FPS: 69, D: 450996 || D: 0, E: 0`\
`WLAN S: 761616, R: 350, E: 0, D: 0, % : 0 || FPS: 69, D: 449347 || D: 0, E: 0`

### Raspberry Pi ground station:
- I use a Raspberry Pi 4 but any 64bit board and OS should work.

sudo apt update && sudo apt upgrade
sudo apt install libdrm-dev libgbm-dev libgles2-mesa-dev libpcap-dev libturbojpeg0-dev libts-dev libsdl2-dev libfreetype6-dev
sudo apt-get install -y aircrack-ng
sudo apt-get install dkms
sudo apt install -y raspberrypi-kernel-headers build-essential bc dkms git

if using a 32bit OS:
sudo nano /boot/config.txt
add " arm_64bit=0" to bottom of file save and reboot

mkdir -p ~/src
cd ~/src
git clone https://github.com/morrownr/8812au-20210629.git
cd ~/src/8812au-20210629
sudo ./install-driver.sh

config = NO
restart = YES

git clone https://github.com/RomanLut/esp32-cam-fpv.git
cd ./esp32-cam-fpv/gs
git submodule update --init --recursive
make -j4

sudo nano /home/pi/fpv.sh

    #!/bin/bash

    echo "ESP32-FPV STARTUP"
    cd /home/pi/esp32-cam-fpv
    cd gs
    sudo airmon-ng check kill
    sudo ip link set wlan1 down
    sudo iw dev wlan1 set type monitor
    sudo ip link set wlan1 up
    sudo ./gs -fullscreen 1 -sm 1 -rx wlan1 -tx wlan1

save

sudo chmod +x /home/pi/fpv.sh

mkdir /home/pi/.config/autostart
nano /home/pi/.config/autostart/fpv.desktop

    [Desktop Entry]
    Type=Application
    Name=FPV
    Exec=/usr/bin/bash /home/pi/fpv.sh

save

VSync is disabled and on a PI4 you should get > 200FPS if everything went ok and you have configured the PI correctly.




