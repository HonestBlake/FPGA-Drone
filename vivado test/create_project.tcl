# From the Vivado Tcl console: source {C:/path/to/FPGA Lab/vivado test/create_project.tcl}
# Or: vivado -mode batch -source "vivado test/create_project.tcl"
set test_dir [file dirname [file normalize [info script]]]
create_project radio_display_test [file join $test_dir build] -part xc7a35tcpg236-1
add_files -norecurse [glob [file join $test_dir src *.v]]
add_files -fileset constrs_1 -norecurse [list [file join $test_dir constraints Basys3-RadioDisplayTest.xdc]]
set_property top RadioDisplayTest [get_filesets sources_1]
add_files -fileset sim_1 -norecurse [list [file join $test_dir sim RadioDisplayTest_tb.v]]
set_property top RadioDisplayTest_tb [get_filesets sim_1]
set_property xsim.simulate.runtime all [get_filesets sim_1]
update_compile_order -fileset sources_1
update_compile_order -fileset sim_1
puts "Created standalone Basys3 project: [file join $test_dir build radio_display_test.xpr]"
