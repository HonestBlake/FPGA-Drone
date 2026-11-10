set test_dir [file dirname [file normalize [info script]]]
create_project radio_motor_test [file join $test_dir build] -part xc7a35tcpg236-1
add_files -norecurse [glob [file join $test_dir src *.v]]
add_files -fileset constrs_1 -norecurse [list [file join $test_dir constraints Basys3-RadioMotorTest.xdc]]
set_property top RadioMotorTest [get_filesets sources_1]
add_files -fileset sim_1 -norecurse [list [file join $test_dir sim RadioMotorTest_tb.v]]
set_property top RadioMotorTest_tb [get_filesets sim_1]
set_property xsim.simulate.runtime all [get_filesets sim_1]
update_compile_order -fileset sources_1
update_compile_order -fileset sim_1
