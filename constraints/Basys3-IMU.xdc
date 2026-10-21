# Proposed IMU assignment using JA1..4. Merge only when these pins are free.
# Header mapping from Digilent's official Basys-3-Master.xdc:
# https://github.com/Digilent/digilent-xdc/blob/master/Basys-3-Master.xdc
# Breakout VCC -> JA6 (3.3 V), GND -> JA5. Use SEN-15335 SPI header.
# This fragment supplies pin assignments only. The board wrapper must provide
# the clock/reset and validate external SPI IO timing including level shifters.
set_property -dict { PACKAGE_PIN J1 IOSTANDARD LVCMOS33 } [get_ports imu_spi_cs_n]
set_property -dict { PACKAGE_PIN L2 IOSTANDARD LVCMOS33 } [get_ports imu_spi_sclk]
set_property -dict { PACKAGE_PIN J2 IOSTANDARD LVCMOS33 } [get_ports imu_spi_mosi]
set_property -dict { PACKAGE_PIN G2 IOSTANDARD LVCMOS33 PULLDOWN true } [get_ports imu_spi_miso]
