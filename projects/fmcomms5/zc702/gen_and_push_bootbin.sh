cp fmcomms5_zc702.runs/impl_1/system_top.bit .
bootgen -arch zynq -image zynq.bif -o BOOT.BIN -w
scp BOOT.BIN root@10.0.0.160:/boot/BOOT.BIN