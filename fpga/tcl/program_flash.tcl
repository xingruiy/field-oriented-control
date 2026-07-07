# ============================================================================
# program_flash.tcl - program the Arty S7-50 QSPI configuration flash.
#
#   vivado -mode batch -source tcl/program_flash.tcl
#
# This is persistent across power cycles and FPGA reconfiguration, unlike
# program.tcl, which loads the FPGA configuration SRAM directly over JTAG.
# ============================================================================

set SCRIPT_DIR [file dirname [file normalize [info script]]]
set ROOT [file normalize $SCRIPT_DIR/..]
set BIT  [file normalize [file join $ROOT build impl foc_top.bit]]
set MCS  [file normalize [file join $ROOT build impl foc_top.mcs]]

proc fail {msg} {
    puts stderr "ERROR: $msg"
    catch {close_hw_manager}
    exit 1
}

puts "=== bitstream: $BIT ==="
puts "=== cfgmem:    $MCS ==="

if {![file exists $BIT]} {
    fail "bitstream not found: $BIT - run 'make build' first, or use 'make flash'"
}

write_cfgmem -force -format mcs -interface SPIx4 -size 16 \
    -loadbit "up 0x0 $BIT" $MCS

open_hw_manager
connect_hw_server

if {[catch {refresh_hw_server} msg]} {
    puts "WARNING: refresh_hw_server failed: $msg"
}

set targets [get_hw_targets -quiet *]
if {[llength $targets] == 0} {
    fail "no hardware targets found - check board power, USB-JTAG cable, and udev permissions"
}
current_hw_target [lindex $targets 0]
open_hw_target

set dev [lindex [get_hw_devices xc7s50*] 0]
if {$dev eq ""} {
    fail "no xc7s50 device found on hardware target [current_hw_target]"
}
current_hw_device $dev

set parts [get_cfgmem_parts -quiet {s25fl128sxxxxxx0-spi-x1_x2_x4}]
if {[llength $parts] == 0} {
    fail "cfgmem part s25fl128sxxxxxx0-spi-x1_x2_x4 not found in this Vivado install"
}

create_hw_cfgmem -hw_device $dev [lindex $parts 0]
set cfg [current_hw_cfgmem]

set_property PROGRAM.FILES $MCS $cfg
set_property PROGRAM.ADDRESS_RANGE {use_file} $cfg
set_property PROGRAM.BLANK_CHECK 0 $cfg
set_property PROGRAM.ERASE 1 $cfg
set_property PROGRAM.CFG_PROGRAM 1 $cfg
set_property PROGRAM.VERIFY 1 $cfg

program_hw_cfgmem -hw_cfgmem $cfg
boot_hw_device $dev

puts "=== programmed QSPI flash from $MCS ==="
close_hw_manager
