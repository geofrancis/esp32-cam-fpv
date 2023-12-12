sudo cp brcmfmac.ko /lib/modules/5.10.103-v7+/kernel/drivers/net/wireless/broadcom/brcm80211/brcmfmac/brcmfmac.ko
sudo cp brcmfmac43436-sdio.bin /usr/lib/firmware/brcm/
depmod -a
