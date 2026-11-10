# Programs the connected Basys3 persistently, then starts the tested image.
set test_dir [file dirname [file normalize [info script]]]
set build_dir [file join $test_dir build]
set bit_file [file join $build_dir flight_test.runs impl_1 FlightTest.bit]
if {![file exists $bit_file]} { error "Build FlightTest first" }
set image_file [file join $build_dir FlightTest.mcs]
# write_cfgmem's loadbit parser requires a path without spaces. Working from
# build provides that path without relocating or changing the source bitstream.
set saved_dir [pwd]
cd $build_dir
write_cfgmem -force -format mcs -size 4 -interface SPIx1 \
    -loadbit {up 0x00000000 flight_test.runs/impl_1/FlightTest.bit} \
    -file FlightTest.mcs
cd $saved_dir

open_hw_manager
connect_hw_server -allow_non_jtag
set targets [get_hw_targets]
if {[llength $targets] != 1} { error "Expected exactly one Basys3 JTAG target" }
set target [lindex $targets 0]
if {![string match */Digilent/210183B31796A $target]} {
    error "Connected target differs from this test's Basys3: $target"
}
current_hw_target $target
open_hw_target
set devices [get_hw_devices -filter {PART =~ "xc7a35t*"}]
if {[llength $devices] != 1} { error "Expected exactly one xc7a35t FPGA" }
set fpga [lindex $devices 0]
current_hw_device $fpga
refresh_hw_device -update_hw_probes false $fpga
set mode0 [get_property {REGISTER.CONFIG_STATUS.BIT08_MODE_PIN_M[0]} $fpga]
set mode1 [get_property {REGISTER.CONFIG_STATUS.BIT09_MODE_PIN_M[1]} $fpga]
set mode2 [get_property {REGISTER.CONFIG_STATUS.BIT10_MODE_PIN_M[2]} $fpga]
puts "BOOT_MODE_M2_M1_M0: $mode2$mode1$mode0"

set flash_part [get_cfgmem_parts mx25l3273f-spi-x1_x2_x4]
if {[llength $flash_part] < 1} { error "Required Macronix flash support not found" }
puts "FLASH_CANDIDATES: $flash_part"
create_hw_cfgmem -hw_device $fpga [lindex $flash_part 0]
set flash [get_property PROGRAM.HW_CFGMEM $fpga]
set_property PROGRAM.ADDRESS_RANGE use_file $flash
set_property PROGRAM.FILES [list $image_file] $flash
set_property PROGRAM.BLANK_CHECK 0 $flash
set_property PROGRAM.ERASE 0 $flash
set_property PROGRAM.CFG_PROGRAM 0 $flash
set_property PROGRAM.VERIFY 0 $flash
create_hw_bitstream -hw_device $fpga [get_property PROGRAM.HW_CFGMEM_BITFILE $fpga]
program_hw_devices $fpga

# Readback checks the actual flash identification before any erase/write.
set backup_file [file join $build_dir flash_before_flight_test_[clock format [clock seconds] -format %Y%m%d_%H%M%S].bin]
readback_hw_cfgmem -all -format bin -file $backup_file $flash
if {![file exists $backup_file] || [file size $backup_file] != 4194304} {
    error "Incomplete 4 MB flash backup; flash not erased"
}
puts "FLASH_BACKUP: $backup_file"
set_property PROGRAM.BLANK_CHECK 1 $flash
set_property PROGRAM.ERASE 1 $flash
set_property PROGRAM.CFG_PROGRAM 1 $flash
set_property PROGRAM.VERIFY 1 $flash
program_hw_cfgmem $flash
puts "PASS: FlightTest configuration flash programmed and verified."

if {"$mode2$mode1$mode0" eq "001"} {
    boot_hw_device $fpga
    set booted 0
    for {set attempt 0} {$attempt < 30} {incr attempt} {
        after 1000
        refresh_hw_device -update_hw_probes false $fpga
        if {[get_property REGISTER.CONFIG_STATUS.BIT14_DONE_PIN $fpga] eq "1"} {
            set booted 1
            break
        }
    }
    if {!$booted} { error "Flash verified, but FPGA did not finish QSPI boot" }
    puts "PASS: FPGA booted FlightTest from configuration flash (DONE=1)."
} else {
    set_property PROGRAM.FILE $bit_file $fpga
    program_hw_devices $fpga
    refresh_hw_device -update_hw_probes false $fpga
    if {[get_property REGISTER.CONFIG_STATUS.BIT14_DONE_PIN $fpga] ne "1"} {
        error "Flash verified, but JTAG startup did not finish"
    }
    puts "PASS: FlightTest running through JTAG (DONE=1)."
    puts "MANUAL_STEP: Power off and move JP1 MODE to QSPI for automatic startup."
}
close_hw_target
disconnect_hw_server
close_hw_manager

