READEME by Serge Malo

## ZYNQ ZC702 + FMCOMMS5 Project
### How to build
Make sure Vivado GUI app is closed.
In a terminal:
```
cd zc702
source adi_build_env.sh
rm -rf ~/w/adi-hdl/ipcache
make clean
make
```
### How to load the image to the FPGA
(I will write a script to automate this process)

1. Build a new BOOT.BIN  
1.1 Copy your new bit file (fmcomms5_zc702.runs/impl_1/system_top.bit) to ./zc702  
1.2 Call bootgen:  
```
$ bootgen -arch zynq -image zynq.bif -o BOOT.BIN -w
```
2. Upload the new BOOT.BIN to the device (overwrite the old one)
```
scp BOOT.BIN root@10.0.0.160:/boot/BOOT.BIN
```
3. On the device, sync+reboot
```
$ sync
$ reboot
```
4. After reboot, verify the new load is active on the device
```
$ dmesg | grep -i branch
[    2.019206] axi_sysid 45000000.axi-sysid-0: [fmcomms5] on [zc702] git branch <2023_R2_p1_SM> git <d146370c10fdd55156de2bafdd9b24292c01b6e1> dirty [2026-07-06 20:29:26] UTC
```

---
(Original READEME from ADI)
# FMCOMMS5 HDL Project

Here are some pointers to help you:
  * [Board Product Page](https://www.analog.com/eval-ad-fmcomms5-ebz)
  * Parts : [RF Agile Transceiver](https://www.analog.com/ad9361)
  * Project Doc: https://wiki.analog.com/resources/eval/user-guides/ad-fmcomms5-ebz
  * HDL Doc: https://wiki.analog.com/resources/eval/user-guides/ad-fmcomms5-ebz/hardware
  * Linux Drivers: https://wiki.analog.com/resources/eval/user-guides/ad-fmcomms2-ebz/software/linux
