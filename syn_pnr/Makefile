export

# Make file for TSMC 65RF place and route 
# Master Verilog file
TOP_LEVEL	= ibex_chiptop
# svt standard cells
PDK_HOME	= $(TSMC65RF_PDK_IC6)/stdcell_dig/fb_tsmc065gp_rvt_lvt
VCS_HOME 	= /software/Synopsys-2021_x86_64/vcs/R-2020.12-SP1-1
VERILOG_LIB	= $(PDK_HOME)/aci/sc-ad10/verilog/tsmc65_rvt_sc_adv10.v
PROJECT_HOME	= .
OUT_DIR		= $(PROJECT_HOME)

TSMC_DB_DIR		= $(PDK_HOME)/aci/sc-ad10/synopsys
# TSMC_DB			= scadv10_cln65gp_rvt_ss_0p9v_125c.db
TSMC_DB			= scadv10_cln65gp_rvt_tt_1p0v_25c.db
TSMC_DB_BEST	= scadv10_cln65gp_rvt_ff_1p1v_m40c.db
# TSMC_LIB		= $(TSMC_DB_DIR)/scadv10_cln65gp_rvt_ss_0p9v_125c.lib
TSMC_LIB		= $(TSMC_DB_DIR)/scadv10_cln65gp_rvt_tt_1p0v_25c.lib
TSMC_LIB_BEST	= $(TSMC_DB_DIR)/scadv10_cln65gp_rvt_ff_1p1v_m40c.lib
####TODO#####
SRAM_LIB		= $(PROJECT_HOME)/libraries/icache_tag_store_nldm_tt_1p00v_1p00v_25c_syn.lib $(PROJECT_HOME)/libraries/icache_data_store_nldm_tt_1p00v_1p00v_25c_syn.lib $(PROJECT_HOME)/libraries/dmem_sram_nldm_tt_1p00v_1p00v_25c_syn.lib
SRAM_LEF		= $(PROJECT_HOME)/libraries/icache_tag_store.vclef $(PROJECT_HOME)/libraries/icache_data_store.vclef $(PROJECT_HOME)/libraries/dmem_sram.vclef
####TODO#####
LIB_LEF			= $(PDK_HOME)/aci/sc-ad10/lef/tsmc65_rvt_sc_adv10_macro.lef
TECH_LEF		= $(PDK_HOME)/aci/sc-ad10/lef/tsmc_cln65_a10_6X2Z_tech.lef
GDS_LIB			= $(PDK_HOME)/aci/sc-ad10/gds2/tsmc65_rvt_sc_adv10.gds2
CDL_LIB			= $(PDK_HOME)/aci/sc-ad10/lvs_netlist/tsmc65_rvt_sc_adv10.cdl

PNR_DIR			= $(PROJECT_HOME)/pnr
WORK_DIR		= $(PROJECT_HOME)/work
LOG_DIR			= $(PROJECT_HOME)/logs
REPORT_DIR		= $(PROJECT_HOME)/reports
SDC_OUT_DIR		= $(PROJECT_HOME)/sdcout
SDF_OUT_DIR		= $(PROJECT_HOME)/sdfout
VLOGOUT_DIR		= $(PROJECT_HOME)/vlogout

# PNR Cells to take care of RVT/LVT stuff
POWERCAP_CELL  = FILLCAP16A10TR
WELLTAP_CELL   = FILLTIE2A10TR
TIEHILO_CELL   = TIEHIX1MA10TR TIELOX1MA10TR
DESIGN_CLOCK   = clk_i
DESIGN_RESET   = rst_ni
FILLER_CELL    = "FILL128A10TR FILL64A10TR FILL32A10TR FILL16A10TR FILLCAP8A10TR FILL4A10TR FILL2A10TR FILL1A10TR"
INCPHY_CELL    = FILLCAP16A10TR FILLCAP8A10TR
EXCPHY_CELL    = FILL128A10TR FILLTIE128A10TR FILL64A10TR FILLTIE64A10TR FILL32A10TR FILLTIE32A10TR FILL16A10TR FILLTIE16A10TR FILL8A10TR FILLTIE8A10TR FILL4A10TR FILLTIE4A10TR FILL2A10TR FILLTIE2A10TR FILL1A10TR


chiptop_synth: $(core_synth_script) $(LOG_DIR) $(SDF_OUT_DIR) $(SDC_OUT_DIR) $(VLOGOUT_DIR) $(REPORT_DIR)
	@echo "##################################################################"
	@echo "#     Synthesizing Verilog                                       #"
	@echo "##################################################################"
	SDF_OUT_DIR=$(SDF_OUT_DIR) SDC_OUT_DIR=$(SDC_OUT_DIR) VLOGOUT_DIR=$(VLOGOUT_DIR) TOP_LEVEL=$(TOP_LEVEL) REPORT_DIR=$(REPORT_DIR) SYNOPSYS_DB_DIR=$(TSMC_DB_DIR) SYNOPSYS_DB=$(TSMC_DB) SYNOPSYS_DB_BEST=$(TSMC_DB_BEST) dc_shell-xg-t -f $(PROJECT_HOME)/synthesis_script/chiptop_synth_dc.tcl 2>&1 | tee $(LOG_DIR)/$(TOP_LEVEL)_synthesis.log


chiptop_PNR:
	@echo "##################################################################"
	@echo "#     Starting PnR                                       		#"
	@echo "##################################################################"
	SDF_OUT_DIR=$(SDF_OUT_DIR) GDS_LIB=$(GDS_LIB) LIB_LEF=$(LIB_LEF) TECH_LEF=$(TECH_LEF) SDC_OUT_DIR=$(SDC_OUT_DIR) VLOGOUT_DIR=$(VLOGOUT_DIR) TIME_LIB=$(TSMC_LIB) TOP_LEVEL=$(TOP_LEVEL) innovus -init pnr/encounter.tcl 2>&1 | tee $(LOG_DIR)/$(TOP_LEVEL)_pnr.log



#make log file directory
$(LOG_DIR):
	@mkdir $(LOG_DIR)


#make report directory
$(REPORT_DIR):
	@mkdir $(REPORT_DIR)


#make synopsys design constraints directory
$(SDC_OUT_DIR):
	@mkdir $(SDC_OUT_DIR)


#make standard delay format directory
$(SDF_OUT_DIR):
	@mkdir $(SDF_OUT_DIR)


#make verilog output directory
$(VLOGOUT_DIR):
	@mkdir $(VLOGOUT_DIR)


#make do file directory
$(DOFILE_DIR):
	@mkdir $(DOFILE_DIR)


#make vector directory
$(VECTOR_DIR):
	@mkdir $(VECTOR_DIR)


clean:
	$(MAKE) -C $(PNR_DIR) clean
	rm -rf $(SDC_OUT_DIR)
	rm -rf $(SDF_OUT_DIR)
	rm -rf $(REPORT_DIR)
	rm -rf $(LOG_DIR)
	rm -rf $(VLOGOUT_DIR)
	rm -rf $(WORK_DIR)
	rm -rf $(DOFILE_DIR)
	rm -rf $(VECTOR_DIR)
	rm -rf $(PROJECT_HOME)/.gate_compiled
	rm -rf $(PROJECT_HOME)/.pnr_complete
	rm -rf $(PROJECT_HOME)/.verilog_lib_compiled
	rm -rf $(PROJECT_HOME)/.verilog_compiled
	rm -rf $(PROJECT_HOME)/alib-52
	rm -rf $(PROJECT_HOME)/command.log
	rm -rf $(PROJECT_HOME)/default.svf
	rm -rf $(PROJECT_HOME)/transcript
	rm -rf $(PROJECT_HOME)/vsim.wlf
	rm -rf $(PROJECT_HOME)/innovus*
	rm -rf $(PROJECT_HOME)/nano*
	rm -rf $(PROJECT_HOME)/timing*

clean_pnr:
	rm -rf .routing_guide.rgf
	rm -rf .tdrlog
	rm -rf .timing_file.tif
	rm -rf cts.rguide
	rm -rf encounter.cmd*
	rm -rf encounter.log*
	rm -rf innovus.cmd*
	rm -rf innovus.log*
	rm -rf final.def
	rm -rf final.dspf
	rm -rf final.gds2
	rm -rf report.*
	rm -rf $(TOP_LEVEL).*
	rm -rf tiehilo.rpt
	rm -rf timingReports
	rm -rf v2lvs.log
	rm -rf .viaViol.*
	rm -rf .clock_group_typView_*
	rm $(LOG_DIR)/*_pnr.log