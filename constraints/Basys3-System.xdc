# System output pin assignments from Digilent's Basys-3-Master.xdc:
# https://github.com/Digilent/digilent-xdc/blob/master/Basys-3-Master.xdc
# Merge into the board wrapper's constraints; this is not a complete Drone XDC.
# uart_data/valid and display_* write ports belong to internal FPGA logic.
# The board wrapper must also supply a synchronized reset and a clock constraint.

# Scaled battery input: VAUX6 on JXADC XA1_P / XA1_N (channel default = 6).
# P receives 0..1 V representing 0..16 V pack; N connects to analog ground.
set_property -dict { PACKAGE_PIN J3 IOSTANDARD LVCMOS33 } [get_ports battery_vin_p]
set_property -dict { PACKAGE_PIN K3 IOSTANDARD LVCMOS33 } [get_ports battery_vin_n]

# Onboard LD0 for beacon timing demonstration (active high).
set_property -dict { PACKAGE_PIN U16 IOSTANDARD LVCMOS33 } [get_ports beacon_led]

# FPGA TX -> onboard USB-UART bridge -> host COM port (PROG/J4 USB connector).
set_property -dict { PACKAGE_PIN A18 IOSTANDARD LVCMOS33 } [get_ports uart_tx]

# Segment cathodes A..G and decimal point, all active low.
set_property -dict { PACKAGE_PIN W7 IOSTANDARD LVCMOS33 } [get_ports {seg[0]}]
set_property -dict { PACKAGE_PIN W6 IOSTANDARD LVCMOS33 } [get_ports {seg[1]}]
set_property -dict { PACKAGE_PIN U8 IOSTANDARD LVCMOS33 } [get_ports {seg[2]}]
set_property -dict { PACKAGE_PIN V8 IOSTANDARD LVCMOS33 } [get_ports {seg[3]}]
set_property -dict { PACKAGE_PIN U5 IOSTANDARD LVCMOS33 } [get_ports {seg[4]}]
set_property -dict { PACKAGE_PIN V5 IOSTANDARD LVCMOS33 } [get_ports {seg[5]}]
set_property -dict { PACKAGE_PIN U7 IOSTANDARD LVCMOS33 } [get_ports {seg[6]}]
set_property -dict { PACKAGE_PIN V7 IOSTANDARD LVCMOS33 } [get_ports dp]

# Digit enables AN0 (rightmost) through AN3 (leftmost), all active low.
set_property -dict { PACKAGE_PIN U2 IOSTANDARD LVCMOS33 } [get_ports {an[0]}]
set_property -dict { PACKAGE_PIN U4 IOSTANDARD LVCMOS33 } [get_ports {an[1]}]
set_property -dict { PACKAGE_PIN V4 IOSTANDARD LVCMOS33 } [get_ports {an[2]}]
set_property -dict { PACKAGE_PIN W4 IOSTANDARD LVCMOS33 } [get_ports {an[3]}]
