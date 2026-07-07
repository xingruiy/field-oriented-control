# ============================================================================
# program.tcl - program the Arty S7-50 over the Digilent USB-JTAG.
#
#   vivado -mode batch -source tcl/program.tcl
# ============================================================================

set SCRIPT_DIR [file dirname [file normalize [info script]]]
set ROOT [file normalize $SCRIPT_DIR/..]
set BIT  [file normalize [file join $ROOT build impl foc_top.bit]]

proc fail {msg} {
    puts stderr "ERROR: $msg"
    catch {close_hw_manager}
    exit 1
}

puts "=== bitstream: $BIT ==="

if {![file exists $BIT]} {
    fail "bitstream not found: $BIT - run 'make build' first, or use 'make flash' to build and program"
}

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
set_property PROGRAM.FILE $BIT $dev
program_hw_devices $dev
puts "=== programmed $BIT ==="
close_hw_manager
