#/**************************************************/
#/* Compile Script for Synopsys                    */
#/*                                                */
#/* dc_shell-xg-t -f compile_dc.tcl                */
#/*                                                */
#/* Christopher Edmonds, OSU                       */
#/* edmondsc@onid.orst.edu                         */
#/**************************************************/

pwd

set hdlin_auto_save_templates true
set hdlin_check_no_latch      true
set hdlin_warn_sens_list      true

#/**************************************************/
#/* User Set Variables                             */
#/**************************************************/

#/* Top-level Module Name                          */
set design_toplevel [getenv TOP_LEVEL]

#/* The name of the clock pin. If no clock-pin     */
#/* exists, pick anything                          */
set design_clock_pin [getenv DESIGN_CLOCK]

#/* The name of the reset pin. If no clock-pin     */
#/* exists, pick anything                          */
set design_reset_pin [getenv DESIGN_RESET]

#/* Max target frequency in MHz for optimization       */
set design_clk_freq_MHz 50

# Input delay and output delay are specified to describe delay outside of your design
#/* Delay of input signals                         */
# Specifies the input delay relative to the clock edge.
# This usually represents previous blocks’ clock to Q delay + combinational logic delay + Route delay with respect to clk edge
set design_input_delay_ns 4

#/* Reserved time for output signals               */
# Specifies the required time of the output relative to the clock edge.
# can be calculated as below.
# set_output_delay -max = max(Tdelay,out2ext) + Tsetup,ext
# set_output_delay -min = min(Tdelay,out2ext) + Thold,ext
set design_output_delay_ns 4

#/* Maximum delay on the specified path in the current design..
# Allows you to specify the max path length for any start point to any endpoint.
# set design_max_delay_ns 0.5

#/**************************************************/
#/* Shouldn't Need To Change the Rest              */
#/**************************************************/
set SynopsysInstall [getenv "DCROOT"]
set search_path [list . \
[format "%s%s" $SynopsysInstall /libraries/syn] \
[format "%s%s" $SynopsysInstall /dw/sim_ver] \
]
#set synlib_wait_for_design_license [list "DesignWare-Foundation"]
set symbol_library [list generic.sdb]
set synthetic_library [list dw_foundation.sldb]
#set synthetic_library dw_foundation.sldb

define_design_lib WORK -path ./work

define_name_rules nameRules -restricted "!@#$%^&*()\\-" -case_insensitive

set cell_path [getenv SYNOPSYS_DB_DIR]
set search_path [concat $search_path $cell_path]

#set alib_library_analysis_path $cell_path
set alib_library_analysis_path ./

set verilogout_show_unconnected_pins "true"
#set_ultra_optimization true
#set_ultra_optimization -force    

set target_library [format "%s" [getenv SYNOPSYS_DB]]

#set link_library    "* $target_library"
set link_library [concat [concat "*" $target_library] $synthetic_library]

#######*********TODO**************###########
#######*********TODO**************###########
#######*********TODO**************###########
set search_path [list . ${search_path} "../hdl/ibex_core/rtl/" "../hdl/ibex_core/package/" ]
# ../src/aes/package/ ../src/aes/rtl/ ../src/irq/rtl/ ../src/mmu/package/ ../src/mmu/rtl/ ../src/packages/ ../src/spi_peripheral/hdl/"]

set dmem_lib libraries/dmem_sram_nldm_tt_1p00v_1p00v_25c_syn.db
set icache_tag_lib libraries/icache_tag_store_nldm_tt_1p00v_1p00v_25c_syn.db
set icache_data_lib libraries/icache_data_store_nldm_tt_1p00v_1p00v_25c_syn.db

set_app_var target_library "${target_library} ${dmem_lib} ${icache_tag_lib} ${icache_data_lib}"
set_app_var link_library "${link_library} ${dmem_lib} ${icache_tag_lib} ${icache_data_lib}"

# analyze -library WORK -format sverilog "../hdl/packages/chiptypes_pkg.sv"

#################################
####### Loads MMU files #########
#################################

set modules {
    "mmu"
}
puts "analyzing mmu"
analyze -library WORK -format sverilog "../hdl/Memory/mmu.sv"



#############################################################
####### Loads ibex_core/rtl and ibex_(module) files #########
#############################################################

############### Ibex Analyze ################
set modules {
    "if_stage" "id_stage" "ex_block" "load_store_unit" "wb_stage"
    "cs_registers" "icache" "compressed_decoder" "decoder"
    "controller" "alu" "multdiv_fast" "csr" "counter" "branch_predict"
    "prefetch_buffer" "fetch_fifo"
}
set rtlprefix "../hdl/ibex_core/rtl"
analyze -library WORK -format sverilog "../hdl/ibex_core/package/ibex_pkg.sv"
analyze -library WORK -format sverilog "../hdl/ibex_core/rtl/ibex_core.sv"
foreach module $modules {
    puts "analyzing $module"
    analyze -library WORK -format sverilog "${rtlprefix}/ibex_${module}.sv"
}
#############################################################
####### Loads ibex_core/prim and prim/(module) files ########
#############################################################

analyze -library WORK -format sverilog "../hdl/ibex_core/package/prim_ram_1p_pkg.sv"
analyze -library WORK -format sverilog "../hdl/ibex_core/package/prim_secded_pkg.sv"
analyze -library WORK -format sverilog "../hdl/ibex_core/package/prim_pkg.sv"
analyze -library WORK -format sverilog "../hdl/ibex_core/rtl/ibex_top.sv"
set modules {
    "prim_clock_gating" "prim_generic_clock_gating" "prim_buf" 
    "prim_generic_buf" "prim_ram_1p" "prim_generic_ram_1p"
}
foreach module $modules {
    puts "analyzing $module"
    analyze -library WORK -format sverilog "${rtlprefix}/prim/${module}.sv"
}
set modules {
    "ibex_register_file_ff" "ibex_register_file_latch" 
}
foreach module $modules {
    puts "analyzing $module"
    analyze -library WORK -format sverilog "${rtlprefix}/${module}.sv"
}
# ############  AES  Analyze ###################
# analyze -library WORK -format sverilog "../src/aes/package/aes_pkg.sv"
# set modules [glob -nocomplain ../src/aes/rtl/*.sv]
# foreach module $modules {
#     puts "analyzing $module"
#     analyze -library WORK -format sverilog "${module}"
# }

# ############  IRQ  Analyze ###################
# analyze -library WORK -format sverilog "../src/irq/rtl/fast_irq_if.sv"

# ############  MMU  Analyze ###################
# analyze -library WORK -format sverilog "../src/mmu/package/mmu_pkg.sv"
# set modules [glob -nocomplain ../src/mmu/rtl/*.sv]
# foreach module $modules {
#     puts "analyzing $module"
#     analyze -library WORK -format sverilog "${module}"
# }

# ############  JTAG  Analyze ###################
# analyze -library WORK -format sverilog "../src/jtag_peripheral/packages/jtag_pkg.sv"
# set modules [glob -nocomplain ../src/jtag_peripheral/hdl/*.sv]
# foreach module $modules {
#     puts "analyzing $module"
#     analyze -library WORK -format sverilog "${module}"
# }
# ############  SPI   Analyze ###################
# set modules [glob -nocomplain ../src/spi_peripheral/hdl/*.sv]
# foreach module $modules {
#     puts "analyzing $module"
#     analyze -library WORK -format sverilog "${module}"
# }
# ############  GPIO  Analyze ###################
# analyze -library WORK -format sverilog "../src/gpio_peripheral/hdl/gpio.sv"

############ Chiptop Analyze #################
# analyze -library WORK -format sverilog "../src/ibex_jtag_arbiter.sv"
analyze -library WORK -format sverilog "../hdl/ibex_core/rtl/ibex_chiptop.sv"

#######*********TODO**************###########
#######*********TODO**************###########
#######*********TODO**************###########

elaborate $design_toplevel
current_design $design_toplevel
check_design >> core_design_check.rpt
link

uniquify 

check_design


#Set the wire load model
set_wire_load_model -name tsmc65_wl10

#Set up the clock period
set design_period [expr 1000.0 / $design_clk_freq_MHz ]
set clk_name $design_clock_pin
create_clock -period $design_period -name my_clk $clk_name

#Set clock jitter to 100ps
# Specifies the uncertainty (skew) of specified clock networks.
# Indicates that uncertainty applies only to setup and hold checks, respectively
set_clock_uncertainty -setup 0.1 my_clk
set_clock_uncertainty -hold  0.1 my_clk

#Set the input and output delay
#########NEED TO LOOK INTO THIS ##################
set_input_delay  $design_input_delay_ns  -clock my_clk [all_inputs]
# @ later set_output_delay $design_output_delay_ns -clock my_clk [all_outputs]

# Removes timing constraints from particular paths. 
# for asyn reset, static digital control bits, etc.
#########NEED TO LOOK INTO THIS ##################
set_false_path -from [get_ports "rst_ni"]
# set_false_path -from [get_ports "div_ctrl"]
set_fix_hold my_clk

#Setup the capacitive load on the output pins (pF)
# set_load 2 [all_outputs]
set_host_options -max_cores 4

set_driving_cell -lib_cell INVX1BA10TR -pin Y [all_inputs]

report_constraints -all_violators

# Compile with hierarchical design
# compile_ultra -gate_clock
compile
compile -gate_clock
#compile -ungroup_all

# Make sure we are at the top level
######## WHATS THISSSSS ##################
set current_design  $design_toplevel
change_names -rules verilog -hierarchy -verbose
change_names -rules nameRules -hierarchy -verbose

# Generate area and constraints reports on the optimized design
report_area > [format "%s%s%s" [getenv REPORT_DIR] "/" $design_toplevel ".gate.area.rpt"]

# Generate timing report for worst case path
report_timing -delay max   >  [format "%s%s%s%s" [getenv REPORT_DIR] "/" $design_toplevel ".gate.delay.rpt"]
report_timing -delay min   >  [format "%s%s%s%s" [getenv REPORT_DIR] "/" $design_toplevel ".gate.hold.rpt"]
report_timing              >  [format "%s%s%s%s" [getenv REPORT_DIR] "/" $design_toplevel ".gate.timing.rpt"]
# report_timing -max_path 50 >> [format "%s%s%s%s" [getenv REPORT_DIR] "/" $design_toplevel ".gate.timing.rpt"]
# report_timing_requirement  >> [format "%s%s%s%s" [getenv REPORT_DIR] "/" $design_toplevel ".gate.timing.rpt"]
# report_constraint          >> [format "%s%s%s%s" [getenv REPORT_DIR] "/" $design_toplevel ".gate.timing.rpt"]
# report_attribute           >> [format "%s%s%s%s" [getenv REPORT_DIR] "/" $design_toplevel ".gate.timing.rpt"]
# report_constraint -all_violators >> [format "%s%s%s%s" [getenv REPORT_DIR] "/" $design_toplevel ".gate.timing.rpt"]
check_design               >> [format "%s%s%s%s" [getenv REPORT_DIR] "/" $design_toplevel ".gate.timing.rpt"]

# Generate other reports
# report_hierarchy >  [format "%s%s%s%s" [getenv REPORT_DIR] "/" $design_toplevel ".gate.hierarchy.rpt"]
# report_resources >  [format "%s%s%s%s" [getenv REPORT_DIR] "/" $design_toplevel ".gate.resources.rpt"]
# report_reference >> [format "%s%s%s%s" [getenv REPORT_DIR] "/" $design_toplevel ".gate.resources.rpt"]
report_cell      >  [format "%s%s%s%s" [getenv REPORT_DIR] "/" $design_toplevel ".gate.cell.rpt"]

# Generate power report
report_power -verbose > [format "%s%s%s%s" [getenv REPORT_DIR] "/" $design_toplevel ".gate.power.rpt"]

# Save the compiled design
write -format verilog -hierarchy -output  [format "%s%s%s%s" [getenv VLOGOUT_DIR] "/" $design_toplevel ".gate.v"]

# Write out the delay information to the sdf file
# SDF: stand format for back annotation timing info
write_sdf [format "%s%s%s%s" [getenv SDF_OUT_DIR] "/" $design_toplevel ".gate.sdf"] 
# SDC: Synopsys Design Constraint file including timing constraints for PnR
write_sdc [format "%s%s%s%s" [getenv SDC_OUT_DIR] "/" $design_toplevel ".gate.sdc"] 
exit 