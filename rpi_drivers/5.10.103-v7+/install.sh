#!/bin/bash
sudo apt-get update && apt-get full-upgrade
sudo apt install raspberrypi-kernel-headers git libgmp3-dev gawk qpdf bison flex make autoconf libtool texinfo
git clone https://github.com/seemoo-lab/nexmon.git
cd nexmon
source setup_env.sh
make
cd patches/bcm43436b0/9_88_4_65/nexmon/
make
make backup-firmware
make install-firmware
cd /home/pi/
cd utilities/nexutil/
make && make install
sudo iw dev wlan0 set power_save off
sudo cp /home/pi/nexmon/patches/bcm43430a1/7_45_41_46/nexmon/brcmfmac_kernel49/brcmfmac.ko /lib/modules/5.10.103-v7+/kernel/drivers/net/wireless/broadcom/brcm80211/brcmfmac/brcmfmac.ko
sudo depmod -a
reboot
