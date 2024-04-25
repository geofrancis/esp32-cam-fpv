# ESP-VTX
Digital,low latency video transmitter based on ESP32   
origin project: https://github.com/RomanLut/esp32-cam-fpv   

## Features:
It uses FEC encoding (2/3 currently configured) with 1400 byte packets and it achieves quite good performance:
* 800k video rate
* 30 FPS at 800x600 with 50-80 ms latency.
* run at any ESP-cam boards(like AI Thinker etc.) with OV2640.

## Theory
The data is received from the camera module as JPEG ar 20MHz I2S clock and passed directly to the wifi module and written to the SD card if the DVR is enabled.
The ESP camera component was modified to send the data as it's received from the DMA instead of frame-by-frame basis. This decreases latency quite significantly (10-20 ms) and reduces the need to allocate full frames in PSRAM.

The wifi data is send using packet injection with configurable rate - from 2 MB CCK to 72MB MCS7 - and power.

The air unit can also record the video straight from the camera to a sd card. The format is a rudimentary MJPEG without any header so when playing back the FPS will be whatever your player will decide.\
There is significant buffering when writing to SD (3MB at the moment) to work around the very regular slowdowns of sd cards.

The JPEG decoding is done with turbojpeg to lower latency and - based on the resolution - can take between 1 and 7 milliseconds.\
It's then uploaded to 3 separate textures as YUV and converted to RGB in a shader.

The link is bi-directional so the ground station can send data to the air unit. At the moment it sends camera and wifi configuration data but I plan to have a full bi-directional serial port for telemetry coming soon.\
The back link uses 64byte packets with a 2/6 FEC encoding (so quite solid) at a low wifi rate (I think 1Mb).

This is very WIP at the moment, but it proves that the ESP32 can definitely stream video with very low latency. 
I plan to use this is a long range micro quad.

https://www.youtube.com/watch?v=UE7nGN3aSVI      
https://www.youtube.com/watch?v=lh8CYKR0TSs     
### esp32 firmware:
Flash ESP32 binary to address 10000 using web flasher or esptool.

### Raspberry Pi ground station:
- I use a Raspberry Pi 4 but any 64bit pi board and OS should work.

sudo apt update && sudo apt upgrade           
sudo apt install libdrm-dev libgbm-dev libgles2-mesa-dev libpcap-dev libturbojpeg0-dev libts-dev libsdl2-dev libfreetype6-dev             
sudo apt-get install -y aircrack-ng            
sudo apt-get install dkms           
sudo apt install -y raspberrypi-kernel-headers build-essential bc dkms git           

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

CTRL + O
ENTER
CTRL + X
                 

sudo chmod +x /home/pi/fpv.sh                    

mkdir /home/pi/.config/autostart                  
nano /home/pi/.config/autostart/fpv.desktop             

    [Desktop Entry]
    Type=Application
    Name=FPV
    Exec=/usr/bin/bash /home/pi/fpv.sh

CTRL + O
ENTER
CTRL + X




