This folder contains prebuilt nextmon drivers for Raspberry Pi Zero 2W with monitor mode support.

For Linux kernel 5.10 only!

(check core name with ```uname -r```)

https://downloads.raspberrypi.org/raspios_armhf/images/raspios_armhf-2021-05-28/
https://downloads.raspberrypi.org/raspios_lite_armhf/images/raspios_lite_armhf-2021-05-28/
https://downloads.raspberrypi.org/raspios_full_armhf/images/raspios_full_armhf-2021-05-28/

Installation
-
./install.sh

Checking
-
```sudo install airckrack-ng```

```sudo install wireshark```

```sudo airmon-ng start wlan0```


=> ```wlan0mon``` should appear

```sudo wireshark```

=> use wireshark to capture packets on ```wlan0mon```

Building
-

For full building instructions check  https://github.com/seemoo-lab/nexmon

Make sure to build ```patches/bcm43430a1/7_45_41_46/nexmon/``` (driver + RPI0W firmware) and ```patches/bcm43436b0/9_88_4_65/nexmon/``` (RPI02W firmware).

Prebuilt binaries for later Linux kernels can also be found in https://github.com/evilsocket/pwnagotchi images.
