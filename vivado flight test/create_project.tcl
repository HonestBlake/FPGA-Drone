set test_dir [file dirname [file normalize [info script]]]
source [file join $test_dir sources.tcl]
create_project flight_test [file join $test_dir build] -part xc7a35tcpg236-1
add_files -norecurse $design_sources
add_files -fileset constrs_1 -norecurse [list [file join $test_dir constraints Basys3-FlightTest.xdc]]
set_property top FlightTest [get_filesets sources_1]
add_files -fileset sim_1 -norecurse [list [file join $test_dir sim FlightTest_tb.v] [file join $test_dir sim Icm20948Model.vh]]
set_property include_dirs [list [file join $test_dir sim]] [get_filesets sim_1]
set_property top FlightTest_tb [get_filesets sim_1]
set_property xsim.simulate.runtime all [get_filesets sim_1]
update_compile_order -fileset sources_1
update_compile_order -fileset sim_1
