
set search_path [list . ${search_path} "../../hdl/ibex/rtl/" "../../hdl/ibex/package/ "]
# ../../hdl/aes/package/ ../../hdl/aes/rtl/ ../../hdl/irq/rtl/ ../../hdl/mmu/package/ ../../hdl/mmu/rtl/ ../../hdl/packages/ ../../hdl/spi_peripheral/hdl/ ../../hdl/"]
set link_library   ../libraries/TSMC40nm_Standard_Library_0.99V_25C_TYP_X1.db
set target_library ../libraries/TSMC40nm_Standard_Library_0.99V_25C_TYP_X1.db
set symbol_library ../libraries/tech_libgeneric.sdb

set dmem_lib ../libraries/dmem_sram_nldm_tt_1p00v_1p00v_25c_syn.db
set icache_tag_lib ../libraries/icache_tag_store_nldm_tt_1p00v_1p00v_25c_syn.db
set icache_data_lib ../libraries/icache_data_store_nldm_tt_1p00v_1p00v_25c_syn.db

set_app_var target_library "${target_library} ${dmem_lib} ${icache_tag_lib} ${icache_data_lib}"
set_app_var link_library "${link_library} ${dmem_lib} ${icache_tag_lib} ${icache_data_lib}"

analyze -library WORK -format sverilog "../../hdl/packages/chiptypes_pkg.sv"
############### Ibex Analyze ################
set modules {
    "if_stage" "id_stage" "ex_block" "load_store_unit" "wb_stage"
    "cs_registers" "icache" "compressed_decoder" "decoder"
    "controller" "alu" "multdiv_fast" "csr" "counter" "branch_predict"
}
set rtlprefix "../hdl/ibex_core/rtl"
analyze -library WORK -format sverilog "../hdl/ibex_core/package/ibex_pkg.sv"
analyze -library WORK -format sverilog "../hdl/ibex_core/rtl/ibex_core.sv"
foreach module $modules {
    puts "analyzing $module"
    analyze -library WORK -format sverilog "${rtlprefix}/ibex_${module}.sv"
}

analyze -library WORK -format sverilog "../hdl/ibex_core/package/prim_ram_1p_pkg.sv"
analyze -library WORK -format sverilog "../hdl/ibex_core/package/prim_secded_pkg.sv"
analyze -library WORK -format sverilog "../hdl/ibex_core/package/prim_pkg.sv"
analyze -library WORK -format sverilog "../hdl/ibex_core/rtl/ibex_top.sv"
set modules {
    "prim_clock_gating" "prim_generic_clock_gating" "prim_buf" 
    "prim_generic_buf"
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
# analyze -library WORK -format sverilog "../../hdl/aes/package/aes_pkg.sv"
# set modules [glob -nocomplain ../../hdl/aes/rtl/*.sv]
# foreach module $modules {
#     puts "analyzing $module"
#     analyze -library WORK -format sverilog "${module}"
# }

# ############  IRQ  Analyze ###################
# analyze -library WORK -format sverilog "../../hdl/irq/rtl/fast_irq_if.sv"

# ############  MMU  Analyze ###################
# analyze -library WORK -format sverilog "../../hdl/mmu/package/mmu_pkg.sv"
# set modules [glob -nocomplain ../../hdl/mmu/rtl/*.sv]
# foreach module $modules {
#     puts "analyzing $module"
#     analyze -library WORK -format sverilog "${module}"
# }

############  JTAG  Analyze ###################
#tbd
############  SPI   Analyze ###################
#tbd

############ Chiptop Analyze #################
analyze -library WORK -format sverilog "../hdl/ibex_core/rtl/ibex_chiptop.sv"

elaborate chiptop -library WORK
check_design >> core_design_check.rpt
link

file mkdir reports
file mkdir results

create_clock -name "clk_i" -period 20 clk_i
set_input_delay 4 -clock clk_i [all_inputs]
set_output_delay 4 -clock clk_i [all_outputs]
compile_ultra
#compile -ungroup_all
uplevel #0 { report_power -analysis_effort high } >> reports/core_power.rpt
uplevel #0 { report_area } >> reports/core_area.rpt
uplevel #0 { report_port } >> reports/core_port.rpt
uplevel #0 { report_timing } >> reports/core_timing.rpt
check_timing
# sizeof_collection [ get_cells -hier * ]
# source core_synth_scan.tcl
