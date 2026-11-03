#!/usr/bin/bash
 
# Step 1: make
# Step 2: unpack bootgen_sysfiles to kuiper-original/
# Backup Original BOOT.BIN
cp boot_v1b/BOOT.BIN ./BOOT.BIN.BAK
rm -rf boot_v1b
# Step 3: vitis
rm -rf build_boot_bin_py
vitis -s gen_boot.py 2>&1 | tee gen_boot.log

# Step 4: assemble BOOT.BIN in a new, empty folder.
# 4.1 - copy files to new empty folder
mkdir boot_v1b && cd boot_v1b

cp ../build_boot_bin_py/hw0/export/hw0/sw/boot/fsbl.elf .                 # NEW, from your .xsa
unzip -o -j ../fmcomms5_zcu102.sdk/system_top.xsa system_top.bit -d .     # NEW, your bitstream
cp ../kuiper-original/pmufw.elf .
cp ../kuiper-original/bl31.elf .
cp ../kuiper-original/u-boot_xilinx_zynqmp_zcu102_revA.elf .
cp ../kuiper-original/regs.init .

# 4.2 Write the recipe file zynq.bif in this folder. It follows the same order as ADI’s working image, plus the register write:
cat > zynq.bif <<'EOF'
the_ROM_image:
{
[init] regs.init
[bootloader,destination_cpu=a53-0] fsbl.elf
[pmufw_image] pmufw.elf
[destination_device=pl] system_top.bit
[destination_cpu=a53-0,exception_level=el-3,trustzone] bl31.elf
[destination_cpu=a53-0,exception_level=el-2] u-boot_xilinx_zynqmp_zcu102_revA.elf
}
EOF

# 4.3 bootegn
bootgen -arch zynqmp -image zynq.bif -o BOOT.BIN -w

# Step 5: Copyt to the board
scp BOOT.BIN root@10.0.0.125:/boot/BOOT.BIN

