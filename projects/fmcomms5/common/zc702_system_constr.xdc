###############################################################################
## Copyright (C) 2014-2023 Analog Devices, Inc. All rights reserved.
### SPDX short identifier: ADIBSD
###############################################################################

# constraints

# iic

set_property  -dict {PACKAGE_PIN  W11   IOSTANDARD LVCMOS25 PULLTYPE PULLUP} [get_ports iic_scl]
set_property  -dict {PACKAGE_PIN  W8    IOSTANDARD LVCMOS25 PULLTYPE PULLUP} [get_ports iic_sda]

# gpio (switches, leds and such)

set_property  -dict {PACKAGE_PIN  G19   IOSTANDARD LVCMOS25} [get_ports gpio_bd[0]]   ; ## GPIO_SW_N
set_property  -dict {PACKAGE_PIN  F19   IOSTANDARD LVCMOS25} [get_ports gpio_bd[1]]   ; ## GPIO_SW_S
set_property  -dict {PACKAGE_PIN  W6    IOSTANDARD LVCMOS25} [get_ports gpio_bd[2]]   ; ## GPIO_DIP_SW0
set_property  -dict {PACKAGE_PIN  W7    IOSTANDARD LVCMOS25} [get_ports gpio_bd[3]]   ; ## GPIO_DIP_SW1
set_property  -dict {PACKAGE_PIN  H17   IOSTANDARD LVCMOS25} [get_ports gpio_bd[4]]   ; ## XADC_GPIO_0
set_property  -dict {PACKAGE_PIN  H22   IOSTANDARD LVCMOS25} [get_ports gpio_bd[5]]   ; ## XADC_GPIO_1
set_property  -dict {PACKAGE_PIN  G22   IOSTANDARD LVCMOS25} [get_ports gpio_bd[6]]   ; ## XADC_GPIO_2
set_property  -dict {PACKAGE_PIN  H18   IOSTANDARD LVCMOS25} [get_ports gpio_bd[7]]   ; ## XADC_GPIO_3

# Define SPI clock
create_clock -name spi0_clk      -period 40   [get_pins -hier */EMIOSPI0SCLKO]
create_clock -name spi1_clk      -period 40   [get_pins -hier */EMIOSPI1SCLKO]
