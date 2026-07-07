# ============================================================================
# arty_s7.xdc - constraints for 01-pwm-gen on the Arty S7-50 (xc7s50csga324-1).
#
# Pins verified against the Digilent Arty-S7-50 master XDC and the sibling
# ../fpga project. Only the ports used by pwm_top are constrained.
# ============================================================================

# ---- 100 MHz oscillator -----------------------------------------------------
set_property -dict { PACKAGE_PIN R2 IOSTANDARD SSTL135 } [get_ports clk100]
create_clock -name sys_clk -period 10.000 [get_ports clk100]

# Bank 34 is a 1.35 V DDR bank: clk100 (R2) and sw[3] (M5) are SSTL135 and need
# a VREF. M5 is the bank's VREF site, so generate VREF internally to free it.
set_property INTERNAL_VREF 0.675 [get_iobanks 34]

# ---- reset button (active low) ----------------------------------------------
set_property -dict { PACKAGE_PIN C18 IOSTANDARD LVCMOS33 } [get_ports ck_rstn]
set_false_path -from [get_ports ck_rstn]

# ---- slide switches: duty select (sw[3:0]) ----------------------------------
set_property -dict { PACKAGE_PIN H14 IOSTANDARD LVCMOS33 } [get_ports {sw[0]}]
set_property -dict { PACKAGE_PIN H18 IOSTANDARD LVCMOS33 } [get_ports {sw[1]}]
set_property -dict { PACKAGE_PIN G18 IOSTANDARD LVCMOS33 } [get_ports {sw[2]}]
set_property -dict { PACKAGE_PIN M5  IOSTANDARD SSTL135 } [get_ports {sw[3]}]
set_false_path -from [get_ports {sw[*]}]

# ---- PWM outputs ------------------------------------------------------------
# Pmod JA pin 1 (scope probe) and LED0 (visual). JA pin 5/11 are GND for the probe.
set_property -dict { PACKAGE_PIN L17 IOSTANDARD LVCMOS33 } [get_ports pwm_ja]
set_property -dict { PACKAGE_PIN E18 IOSTANDARD LVCMOS33 } [get_ports pwm_led]

# ---- bitstream config -------------------------------------------------------
set_property CFGBVS VCCO        [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
