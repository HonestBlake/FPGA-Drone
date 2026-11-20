# Basys3 rev B/C, xc7a35tcpg236-1. Pin reference:
# https://github.com/Digilent/digilent-xdc/blob/master/Basys-3-Master.xdc
set_property -dict { PACKAGE_PIN W5 IOSTANDARD LVCMOS33 } [get_ports clk]
create_clock -period 10.000 -name sys_clk [get_ports clk]
create_generated_clock -name logic_clk -source [get_ports clk] -divide_by 10 -invert [get_pins -hierarchical -filter {NAME =~ *core_phase_reg/Q}]

# Receiver servo iBUS TX -> Pmod JB1 -> FPGA input tx.
set_property -dict { PACKAGE_PIN A14 IOSTANDARD LVCMOS33 PULLUP true } [get_ports tx]
set_property -dict { PACKAGE_PIN U18 IOSTANDARD LVCMOS33 } [get_ports btnC]
set_property -dict { PACKAGE_PIN V17 IOSTANDARD LVCMOS33 } [get_ports {sw[0]}]
set_property -dict { PACKAGE_PIN V16 IOSTANDARD LVCMOS33 } [get_ports {sw[1]}]
set_property -dict { PACKAGE_PIN W16 IOSTANDARD LVCMOS33 } [get_ports {sw[2]}]
set_property -dict { PACKAGE_PIN W17 IOSTANDARD LVCMOS33 } [get_ports {sw[3]}]
set_property -dict { PACKAGE_PIN W15 IOSTANDARD LVCMOS33 } [get_ports {sw[4]}]

set_property -dict { PACKAGE_PIN W7 IOSTANDARD LVCMOS33 } [get_ports {seg[0]}]
set_property -dict { PACKAGE_PIN W6 IOSTANDARD LVCMOS33 } [get_ports {seg[1]}]
set_property -dict { PACKAGE_PIN U8 IOSTANDARD LVCMOS33 } [get_ports {seg[2]}]
set_property -dict { PACKAGE_PIN V8 IOSTANDARD LVCMOS33 } [get_ports {seg[3]}]
set_property -dict { PACKAGE_PIN U5 IOSTANDARD LVCMOS33 } [get_ports {seg[4]}]
set_property -dict { PACKAGE_PIN V5 IOSTANDARD LVCMOS33 } [get_ports {seg[5]}]
set_property -dict { PACKAGE_PIN U7 IOSTANDARD LVCMOS33 } [get_ports {seg[6]}]
set_property -dict { PACKAGE_PIN V7 IOSTANDARD LVCMOS33 } [get_ports dp]
set_property -dict { PACKAGE_PIN U2 IOSTANDARD LVCMOS33 } [get_ports {an[0]}]
set_property -dict { PACKAGE_PIN U4 IOSTANDARD LVCMOS33 } [get_ports {an[1]}]
set_property -dict { PACKAGE_PIN V4 IOSTANDARD LVCMOS33 } [get_ports {an[2]}]
set_property -dict { PACKAGE_PIN W4 IOSTANDARD LVCMOS33 } [get_ports {an[3]}]

# Only asynchronous input -> first synchronizer stage paths are excepted.
# Paths between synchronizer stages remain timed against the 10 MHz logic clock.
set_false_path -from [get_ports tx] -to [get_cells -hierarchical -filter {NAME =~ *rx_meta_reg}]
set_false_path -from [get_ports btnC] -to [get_cells -hierarchical -filter {NAME =~ *reset_meta_reg}]
set_false_path -from [get_ports {sw[4]}] -to [get_cells -hierarchical -filter {NAME =~ *arm_meta_reg}]
set_false_path -from [get_ports {sw[*]}] -to [get_cells -hierarchical -filter {NAME =~ *sw_meta_reg*}]

set_property CONFIG_VOLTAGE 3.3 [current_design]
set_property CFGBVS VCCO [current_design]

# ESC order: JA7 back-left, JA8 front-left, JA9 back-right, JA10 front-right.
set_property -dict { PACKAGE_PIN H1 IOSTANDARD LVCMOS33 } [get_ports {esc[0]}]
set_property -dict { PACKAGE_PIN K2 IOSTANDARD LVCMOS33 } [get_ports {esc[1]}]
set_property -dict { PACKAGE_PIN H2 IOSTANDARD LVCMOS33 } [get_ports {esc[2]}]
set_property -dict { PACKAGE_PIN G3 IOSTANDARD LVCMOS33 } [get_ports {esc[3]}]

# LD0..LD3 selection, LD4 drive enabled, LD5 startup ready, LD6 RC valid, LD7 heartbeat.
set_property -dict { PACKAGE_PIN U16 IOSTANDARD LVCMOS33 } [get_ports {led[0]}]
set_property -dict { PACKAGE_PIN E19 IOSTANDARD LVCMOS33 } [get_ports {led[1]}]
set_property -dict { PACKAGE_PIN U19 IOSTANDARD LVCMOS33 } [get_ports {led[2]}]
set_property -dict { PACKAGE_PIN V19 IOSTANDARD LVCMOS33 } [get_ports {led[3]}]
set_property -dict { PACKAGE_PIN W18 IOSTANDARD LVCMOS33 } [get_ports {led[4]}]
set_property -dict { PACKAGE_PIN U15 IOSTANDARD LVCMOS33 } [get_ports {led[5]}]
set_property -dict { PACKAGE_PIN U14 IOSTANDARD LVCMOS33 } [get_ports {led[6]}]
set_property -dict { PACKAGE_PIN V14 IOSTANDARD LVCMOS33 } [get_ports {led[7]}]

# User IMU wiring: JA1 /CS, JA2 MISO, JA3 SCLK, JA4 MOSI.
set_property -dict { PACKAGE_PIN J1 IOSTANDARD LVCMOS33 } [get_ports imu_spi_cs_n]
set_property -dict { PACKAGE_PIN L2 IOSTANDARD LVCMOS33 PULLDOWN true } [get_ports imu_spi_miso]
set_property -dict { PACKAGE_PIN J2 IOSTANDARD LVCMOS33 } [get_ports imu_spi_sclk]
set_property -dict { PACKAGE_PIN G2 IOSTANDARD LVCMOS33 } [get_ports imu_spi_mosi]

# Additional flight/IMU status LEDs.
set_property -dict { PACKAGE_PIN V13 IOSTANDARD LVCMOS33 } [get_ports {led[8]}]
set_property -dict { PACKAGE_PIN V3 IOSTANDARD LVCMOS33 } [get_ports {led[9]}]
set_property -dict { PACKAGE_PIN W3 IOSTANDARD LVCMOS33 } [get_ports {led[10]}]
set_property -dict { PACKAGE_PIN U3 IOSTANDARD LVCMOS33 } [get_ports {led[11]}]
set_property -dict { PACKAGE_PIN P3 IOSTANDARD LVCMOS33 } [get_ports {led[12]}]
set_property -dict { PACKAGE_PIN N3 IOSTANDARD LVCMOS33 } [get_ports {led[13]}]
set_property -dict { PACKAGE_PIN P1 IOSTANDARD LVCMOS33 } [get_ports {led[14]}]
set_property -dict { PACKAGE_PIN L1 IOSTANDARD LVCMOS33 } [get_ports {led[15]}]

