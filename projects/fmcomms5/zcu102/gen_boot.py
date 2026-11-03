# gen_boot.py - build FSBL + PMUFW from the XSA with the Vitis 2026.1 Python API (replaces ADI's xsct step)
import os, vitis

xsa = os.path.abspath("fmcomms5_zcu102.sdk/system_top.xsa")
ws  = os.path.abspath("build_boot_bin_py")

client = vitis.create_client()
client.set_workspace(path=ws)
plat = client.create_platform_component(
    name="hw0", hw_design=xsa, os="standalone", cpu="psu_cortexa53_0")
plat.build()          # also builds the boot components: FSBL and PMU firmware
vitis.dispose()