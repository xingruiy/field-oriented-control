# ============================================================================
# arty_s7.xdc - constraints for 02-can-bus on the Arty S7-50.
#
# Wiring convention:
#   JA1 -> HVD230 D/TXD
#   JA2 <- HVD230 R/RXD
#   JA3 -> HVD230 RS, driven low by RTL
#   JA5 or JA11 -> HVD230 GND
#   3V3 -> HVD230 VCC
#
# Connect CANH/CANL from the HVD230 to the CANable. Add a 120 ohm termination
# resistor across CANH/CANL at the bench if the transceiver board/CANable do
# not already provide termination.
# ============================================================================

# ---- 100 MHz oscillator -----------------------------------------------------
set_property -dict { PACKAGE_PIN R2 IOSTANDARD SSTL135 } [get_ports clk100]
create_clock -name sys_clk -period 10.000 [get_ports clk100]

set_property INTERNAL_VREF 0.675 [get_iobanks 34]

# ---- reset button (active low) ----------------------------------------------
set_property -dict { PACKAGE_PIN C18 IOSTANDARD LVCMOS33 } [get_ports ck_rstn]
set_false_path -from [get_ports ck_rstn]

# ---- Pmod JA to HVD230 ------------------------------------------------------
set_property -dict { PACKAGE_PIN L17 IOSTANDARD LVCMOS33 } [get_ports can_tx_ja1]
set_property -dict { PACKAGE_PIN L18 IOSTANDARD LVCMOS33 } [get_ports can_rx_ja2]
set_property -dict { PACKAGE_PIN M14 IOSTANDARD LVCMOS33 } [get_ports can_rs_ja3]
set_property PULLUP true [get_ports can_rx_ja2]
set_false_path -from [get_ports can_rx_ja2]

# The demo motor plant updates only when tick is asserted. Its Q15.16
# state-to-state arithmetic has 100000 sys_clk cycles in hardware, not one.
set motor_state_regs [get_cells -hier -regexp {.*u_motor_model/(speed_q|current_q)_reg\[[0-9]+\]}]
set_multicycle_path -setup 100000 -from $motor_state_regs -to $motor_state_regs
set_multicycle_path -hold   99999  -from $motor_state_regs -to $motor_state_regs

# ---- status LEDs ------------------------------------------------------------
set_property -dict { PACKAGE_PIN E18 IOSTANDARD LVCMOS33 } [get_ports {led[0]}]
set_property -dict { PACKAGE_PIN F13 IOSTANDARD LVCMOS33 } [get_ports {led[1]}]
set_property -dict { PACKAGE_PIN E13 IOSTANDARD LVCMOS33 } [get_ports {led[2]}]
set_property -dict { PACKAGE_PIN H15 IOSTANDARD LVCMOS33 } [get_ports {led[3]}]

# ---- bitstream config -------------------------------------------------------
set_property CFGBVS VCCO        [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
