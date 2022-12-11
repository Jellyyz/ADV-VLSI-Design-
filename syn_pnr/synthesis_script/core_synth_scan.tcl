set_scan_configuration -style multiplexed_flip_flop
set test_default_period 100
set_dft_signal -view existing_dft -type ScanClock -timing {45 55} -port clk_i
set_dft_signal -view existing_dft -type Reset -active_state 0 -port rst_ni
set_dft_signal -view spec -type ScanDataIn -port SERIAL_IN
set_dft_signal -view spec -type ScanDataOut -port SERIAL_OUT
set_dft_signal -view spec -type ScanEnable -port SCAN_EN -active_state 1
create_test_protocol
dft_drc

compile_ultra -scan

set_scan_configuration -chain_count 1
set_scan_configuration -clock_mixing no_mix
set_scan_path chain1 -scan_data_in SERIAL_IN -scan_data_out SERIAL_OUT

insert_dft
set_scan_state scan_existing

report_scan_path -view existing_dft -chain all > reports/chain.rep
report_scan_path -view existing_dft -cell all > reports/cell.rep
change_names -hierarchy -rule verilog
write -format verilog -hierarchy -out results/ibex.v
write -format ddc -hierarchy -output results/ibex.ddc
write_scan_def -output results/ibex.def
set test_stil_netlist_format verilog
write_test_protocol -output results/ibex.spf