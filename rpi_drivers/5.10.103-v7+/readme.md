This folder contains prebuilt nextmon drivers for Raspberry Pi Zero 2W with monitor mode support.

For Linux kernel 5.10. only!

https://downloads.raspberrypi.org/raspios_armhf/images/raspios_armhf-2021-05-28/
https://downloads.raspberrypi.org/raspios_lite_armhf/images/raspios_lite_armhf-2021-05-28/
https://downloads.raspberrypi.org/raspios_full_armhf/images/raspios_full_armhf-2021-05-28/

Installation
-
Download ditribution.

```sudo apt-get update & apt-get upgrade```

Check core version:

```uname -r```

It should be 5.10.103-v7+. **It will not work for other core version.***

```./install.sh```

```reboot```

Checking
-
```sudo install airckrack-ng```

```sudo install wireshark```

```sudo airmon-ng start wlan0```

(ignore error)

```iwconfig```

=> ```wlan0mon``` should appear, started in monitor mode.

```sudo wireshark```

=> use wireshark to capture packets on ```wlan0mon```

Building
-

Patched drivers can be built for other kernel versions.

For full building instructions check  https://github.com/seemoo-lab/nexmon 

Make sure to build ```patches/bcm43430a1/7_45_41_46/nexmon/``` which builds driver:```brcmfmac.ko``` and RPI0W firmware, and ```patches/bcm43436b0/9_88_4_65/nexmon/``` which builds RPI02W firmware:```brcmfmac43436-sdio.bin```.

Prebuilt binaries for later Linux kernels can also be found in https://github.com/evilsocket/pwnagotchi images.
