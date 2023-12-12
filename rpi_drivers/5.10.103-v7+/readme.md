This folder contains prebuilt nextmon drivers for Raspberry Pi Zero 2W with monitor mode support.

For full building instructions check  https://github.com/seemoo-lab/nexmon

For Linux kernel core 5.10 only!
(check with uname -r)

https://downloads.raspberrypi.org/raspios_armhf/images/raspios_armhf-2021-05-28/
https://downloads.raspberrypi.org/raspios_lite_armhf/images/raspios_lite_armhf-2021-05-28/
https://downloads.raspberrypi.org/raspios_full_armhf/images/raspios_full_armhf-2021-05-28/


Checking after install:

sudo install airckrack-ng
sudo airmon-ng start wlan0
=> wlan0mon should appear
