set test_dir [file dirname [file normalize [info script]]]
set project_file [file join $test_dir build flight_test.xpr]
if {[file exists $project_file]} {
    open_project $project_file
    reset_run synth_1
} else {
    source [file join $test_dir create_project.tcl]
}
launch_runs synth_1 -jobs 2
wait_on_run synth_1
if {[get_property PROGRESS [get_runs synth_1]] ne "100%"} { error "Synthesis failed" }
launch_runs impl_1 -to_step write_bitstream -jobs 2
wait_on_run impl_1
if {[get_property PROGRESS [get_runs impl_1]] ne "100%"} { error "Implementation failed" }
open_run impl_1
if {[llength [get_clocks logic_clk]] != 1 || [get_property PERIOD [get_clocks logic_clk]] != 100.000} {
    error "Expected the constrained 10 MHz control clock"
}
report_timing_summary -file [file join $test_dir build timing_summary.txt]
report_drc -file [file join $test_dir build drc.txt]
if {[get_property SLACK [get_timing_paths -delay_type max -max_paths 1]] < 0} { error "Setup timing failed" }
if {[get_property SLACK [get_timing_paths -delay_type min -max_paths 1]] < 0} { error "Hold timing failed" }
puts "PASS: FlightTest bitstream generated; setup and hold timing met."

