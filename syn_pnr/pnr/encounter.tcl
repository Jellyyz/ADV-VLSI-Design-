###################################
# Run the design through Encounter
###################################

set init_lef_file "$env(TECH_LEF) $env(LIB_LEF) " ;# Load LEF(Library Exchange Format) Files
# $env(SRAM_LEF)
# Setup design and create floorplan
#loadConfig ./encounter.conf 
################################################################################
# Initial setup : replacement for 'source my_design.global'
set my_toplevel         $env(TOP_LEVEL) 
set init_verilog        $env(VLOGOUT_DIR)/$my_toplevel.gate.v 
set init_top_cell 		$env(TOP_LEVEL)
set init_design_netlisttype {Verilog} ;#Specifies the source of the design netlist.
set init_design_settop {1} ;#specify whether the top cell name is automatically assigned(0) or is set by user(0) 

# In post route stage, default delay calculation does not honor this variable. Slews and delays are computed based on actual parasitics.
set delaycal_use_default_delay_limit {1000}
set delaycal_default_net_delay {1000.0ps}
set delaycal_default_net_load {0.5pf}
set delaycal_input_transition_delay {120.0ps}

set extract_shrink_factor {1.0}
setLibraryUnit -time 1ns
setLibraryUnit -cap 1pf

set init_pwr_net "vdd"
set init_gnd_net "vss"

set init_assign_buffer {0}

set init_mmmc_file pnr/timingSetup.viewDefinition

################################################################################
init_design ;#import the libraries, netlist, and timing environment.

#commitConfig

#Check the design and then save it
saveDesign [format "%s%s" $env(TOP_LEVEL) ".init.enc"]


# Create Floorplan
set width 750
set height 750
set offset 5 

setFPlanMode -enableRectilinearDesign true

floorplan -s $width $height $offset $offset $offset $offset
setObjFPlanPolygon cell $env(TOP_LEVEL) 0  0 $width $height


#######*********TODO**************###########
#######*********TODO**************###########
#######*********TODO**************###########
##CHECKOUT make_pins.py for this
# Partition Flooplan for sub modules
#setObjFPlanBox Module mydesign/switch 255 30 590 380
# #-------------------------------------------------------------------------------------------------
# # Output pins
# editPin -snap TRACK -side INSIDE -layer 6 -assign 0 25  -pin 
editPin -snap TRACK -side INSIDE -layer 6 -assign 0 85  -pin vdd
# editPin -snap TRACK -side INSIDE -layer 6 -assign 0 145  -pin instr_addr\[0\]  
# editPin -snap TRACK -side INSIDE -layer 6 -assign 0 205  -pin instr_addr\[1\] 
# editPin -snap TRACK -side INSIDE -layer 6 -assign 0 265  -pin instr_addr\[2\] 
# editPin -snap TRACK -side INSIDE -layer 6 -assign 0 325  -pin instr_addr\[3\] 
# editPin -snap TRACK -side INSIDE -layer 6 -assign 0 385  -pin instr_addr\[4\] 
# editPin -snap TRACK -side INSIDE -layer 6 -assign 0 445  -pin instr_addr\[5\] 
# editPin -snap TRACK -side INSIDE -layer 6 -assign 0 505  -pin instr_addr\[6\] 
# editPin -snap TRACK -side INSIDE -layer 6 -assign 0 565  -pin instr_addr\[7\] 
# editPin -snap TRACK -side INSIDE -layer 6 -assign 0 625  -pin instr_addr\[8\] 
# editPin -snap TRACK -side INSIDE -layer 6 -assign 0 685  -pin instr_addr\[9\] 
# editPin -snap TRACK -side INSIDE -layer 6 -assign 0 745  -pin instr_addr\[10\] 

# editPin -snap TRACK -side INSIDE -layer 6 -assign 25 750 -pin 
editPin -snap TRACK -side INSIDE -layer 6 -assign 85 750 -pin vss
# editPin -snap TRACK -side INSIDE -layer 6 -assign 145 750 -pin instr_addr\[11\]
# editPin -snap TRACK -side INSIDE -layer 6 -assign 205 750 -pin instr_addr\[12\] 
# editPin -snap TRACK -side INSIDE -layer 6 -assign 265 750 -pin instr_addr\[13\]
# editPin -snap TRACK -side INSIDE -layer 6 -assign 325 750 -pin instr_addr\[14\]
# editPin -snap TRACK -side INSIDE -layer 6 -assign 385 750 -pin instr_rdata\[0\]
# editPin -snap TRACK -side INSIDE -layer 6 -assign 445 750 -pin instr_rdata\[1\]
# editPin -snap TRACK -side INSIDE -layer 6 -assign 505 750 -pin instr_rdata\[2\]
# editPin -snap TRACK -side INSIDE -layer 6 -assign 565 750 -pin instr_rdata\[3\]
# editPin -snap TRACK -side INSIDE -layer 6 -assign 625 750 -pin instr_rdata\[4\]
# editPin -snap TRACK -side INSIDE -layer 6 -assign 685 750 -pin instr_rdata\[5\]
# editPin -snap TRACK -side INSIDE -layer 6 -assign 745 750 -pin instr_rdata\[6\]


 
# editPin -snap TRACK -side INSIDE -layer 6 -assign 25 0 -pin spi_m_miso 
# editPin -snap TRACK -side INSIDE -layer 6 -assign 85 0 -pin spi_m_sclk 
# editPin -snap TRACK -side INSIDE -layer 6 -assign 145 0 -pin spi_s_mosi 
# editPin -snap TRACK -side INSIDE -layer 6 -assign 205 0 -pin spi_s_sclk 
# editPin -snap TRACK -side INSIDE -layer 6 -assign 265 0 -pin spi_s_miso 
# editPin -snap TRACK -side INSIDE -layer 6 -assign 325 0 -pin spi_m_mosi 
editPin -snap TRACK -side INSIDE -layer 6 -assign 385 0 -pin rst_ni
editPin -snap TRACK -side INSIDE -layer 6 -assign 445 0 -pin clk_i
# editPin -snap TRACK -side INSIDE -layer 6 -assign 505 0 -pin irq_extern
# editPin -snap TRACK -side INSIDE -layer 6 -assign 565 0 -pin gpi
# editPin -snap TRACK -side INSIDE -layer 6 -assign 625 0 -pin gpo
# editPin -snap TRACK -side INSIDE -layer 6 -assign 685 0 -pin 
# editPin -snap TRACK -side INSIDE -layer 6 -assign 745 0 -pin phase\[12\]

# editPin -snap TRACK -side INSIDE -layer 6 -assign 750 25  -pin jtag_tdi
# editPin -snap TRACK -side INSIDE -layer 6 -assign 750 85  -pin jtag_tck
# editPin -snap TRACK -side INSIDE -layer 6 -assign 750 145  -pin jtag_tms
# editPin -snap TRACK -side INSIDE -layer 6 -assign 750 205  -pin jtag_trst_n
# editPin -snap TRACK -side INSIDE -layer 6 -assign 750 265  -pin jtag_tdo
# editPin -snap TRACK -side INSIDE -layer 6 -assign 750 325  -pin jtag_tdo_oe
# editPin -snap TRACK -side INSIDE -layer 6 -assign 750 385  -pin jtag_rtck
# # editPin -snap TRACK -side INSIDE -layer 6 -assign 750 445  -pin 
# editPin -snap TRACK -side INSIDE -layer 6 -assign 750 505  -pin instr_rvalid
# editPin -snap TRACK -side INSIDE -layer 6 -assign 750 565  -pin instr_req
# editPin -snap TRACK -side INSIDE -layer 6 -assign 750 625  -pin instr_rdata\[7\]
# editPin -snap TRACK -side INSIDE -layer 6 -assign 750 685  -pin 
# editPin -snap TRACK -side INSIDE -layer 6 -assign 750 745  -pin 


# # Input	pins									
# editPin	-snap	TRACK	-side	INSIDE	-layer	3	-assign	0	1	-pin	clkin
# editPin	-snap	TRACK	-side	INSIDE	-layer	3	-assign	0	1.4	-pin	rstb
# editPin	-snap	TRACK	-side	INSIDE	-layer	3	-assign	0	7.0	-pin	div_ctrl\[0\]
# editPin	-snap	TRACK	-side	INSIDE	-layer	3	-assign	0	7.2	-pin	div_ctrl\[1\]
# editPin	-snap	TRACK	-side	INSIDE	-layer	3	-assign	0	7.4	-pin	div_ctrl\[2\]
# editPin	-snap	TRACK	-side	INSIDE	-layer	3	-assign	0	7.6	-pin	div_ctrl\[3\]
# editPin	-snap	TRACK	-side	INSIDE	-layer	3	-assign	0	7.8	-pin	div_ctrl\[4\]
#-------------------------------------------------------------------------------------------------								
#######*********TODO**************###########
#######*********TODO**************###########
#######*********TODO**************###########

setMultiCpuUsage -localCpu 4

#######*********TODO**************###########
#######*********TODO**************###########
#######*********TODO**************###########
# TO DO FIX SRAM
# placeInstance data_memory 158.41 372.035
# placeInstance u_top/gen_rams_gen_rams_inner_0__data_bank 432.74 5
# placeInstance u_top/gen_rams_gen_rams_inner_0__tag_bank 296.08 5

# createObstruct -cover -inst data_memory -overhang 5 5 5 5
# createObstruct -cover -inst u_top/gen_rams_gen_rams_inner_0__data_bank -overhang 5 5 5 5
# createObstruct -cover -inst u_top/gen_rams_gen_rams_inner_0__tag_bank -overhang 5 5 5 5
# createObstruct 0 360   610.0 750.0
# createObstruct 0.0 80.0   314.0 250.0
# createObstruct 314.0 80.0   460.0 250.0
#######*********TODO**************###########
#######*********TODO**************###########
#######*********TODO**************###########

timeDesign -preplace

# Setup Encounter modes
setPlaceMode -congEffort high
setPlaceMode -clkGateAware false
setPlaceMode -placeIoPins false
setPlaceMode -timingDriven true
setPlaceMode -fp true

#setCTSMode -routeClkNet true
#setCTSMode -routeClkNet true -optAddBuffer true -optLatency true -synthLatencyEffort high -useLibMaxCap true -useLibMaxFanout true -verbose true
#setCTSMode -routeClkNet true -optAddBuffer true -useLibMaxCap true -useLibMaxFanout true
#setCTSMode -routeClkNet true -optAddBuffer true -optLatency true -synthLatencyEffort high -useLibMaxCap true -useLibMaxFanout true -verbose true -engine ck
setOptMode -preserveAssertions			false
setOptMode -leakagePowerEffort			none
#setOptMode -fixHoldAllowSetupTnsDegrade	true
#setOptMode -holdFixingEffort			low
#setOptMode -criticalRange			0.2  (update to next line)
setOptMode -allEndPoints                        true 

setOptMode -fixFanoutLoad			true
setOptMode -holdFixingEffort			high
setOptMode -yieldEffort			high
setOptMode -fixDRC			true
setOptMode -effort				high
setOptMode -fixHoldAllowSetupTnsDegrade		false
#setOptMode -reclaimArea			true
#setOptMode -usefulSkew				true


setMaxRouteLayer 7

#setViaGenOption -invoke_verifyGeometry 1 -create_double_row_cut_via 1 -add_pin_to_pin_via 1 -respect_signal_routes 1 (update to next line!)
# setViaGenMode -invoke_verifyGeometry true -create_double_row_cut_via 1 -add_pin_to_pin_via true -respect_signal_routes	1
setViaGenMode -invoke_verifyGeometry true -add_pin_to_pin_via true -respect_signal_routes	1 -full_cut_via_only true -allow_via_expansion true
#-create_max_row_cut_via

#######*********TODO**************###########
#######*********TODO**************###########
#######*********TODO**************###########
# globalNetConnect vdd -type pgpin -pin vdd -all -verbose 
# globalNetConnect vss -type pgpin -pin vss -all -verbose
# globalNetConnect vdd -pin VDDPE -instanceBasename data_memory -singleInstance data_memory -verbose
# globalNetConnect vdd -pin VDDCE -instanceBasename data_memory -singleInstance data_memory -verbose
# globalNetConnect vss -pin VSSE -instanceBasename data_memory -singleInstance data_memory -verbose
# globalNetConnect vdd -pin VDDPE -instanceBasename u_top/gen_rams_gen_rams_inner_0__data_bank -singleInstance u_top/gen_rams_gen_rams_inner_0__data_bank -verbose
# globalNetConnect vdd -pin VDDCE -instanceBasename u_top/gen_rams_gen_rams_inner_0__data_bank -singleInstance u_top/gen_rams_gen_rams_inner_0__data_bank -verbose
# globalNetConnect vss -pin VSSE -instanceBasename u_top/gen_rams_gen_rams_inner_0__data_bank -singleInstance u_top/gen_rams_gen_rams_inner_0__data_bank -verbose
# globalNetConnect vdd -pin VDDPE -instanceBasename u_top/gen_rams_gen_rams_inner_0__tag_bank -singleInstance u_top/gen_rams_gen_rams_inner_0__tag_bank -verbose
# globalNetConnect vdd -pin VDDCE -instanceBasename u_top/gen_rams_gen_rams_inner_0__tag_bank -singleInstance u_top/gen_rams_gen_rams_inner_0__tag_bank -verbose
# globalNetConnect vss -pin VSSE -instanceBasename u_top/gen_rams_gen_rams_inner_0__tag_bank -singleInstance u_top/gen_rams_gen_rams_inner_0__tag_bank -verbose
globalNetConnect vdd -type tiehi -all -verbose
globalNetConnect vss -type tielo -all -verbose
#######*********TODO**************###########
#######*********TODO**************###########
#######*********TODO**************###########

# Add pwr/gnd rings(addRing) around core
setAddRingOption \
	-avoid_short 1 ;# Specifies whether to prevent a ring or a ring segment from being created if it causes a short violation. 1 to prevent the creation, 0 to enable the creation 
#addRing -spacing_bottom 0.7 -spacing_right 0.7 -spacing_left 0.7 -spacing_top 0.7 -width_left 2.5 -width_bottom 2.5 -width_top 2.5 -width_right 2.5 -layer_bottom M1 -layer_top M1 -layer_right M2 -layer_left M2 -center 1 -around core -nets { vss vdd }

addRing \
	-spacing {top 0.2 bottom 0.2 left 0.2 right 0.2} \
	-width {top 0.6 bottom 0.6 left 0.6 right 0.6} \
	-layer {top M1 bottom M1 left M2 right M2} \
	-center 1 \
	-nets {vss vdd}

# Add pwr/gnd stripe
#setAddStripeMode -break_at {block_ring} ;# Specifies the break point of stripes.
addStripe \
	-block_ring_top_layer_limit M5 \
	-max_same_layer_jog_length 0.56 \
	-padcore_ring_bottom_layer_limit M1 \
	-number_of_sets 40 \
	-padcore_ring_top_layer_limit M4 \
	-spacing 1.84 \
	-merge_stripes_value 0.28 \
	-layer M7 \
	-block_ring_bottom_layer_limit M1 \
	-width 1 \
	-nets { vss vdd } 
# addStripe: additional connections from power rings to pwr/gnd rails in the core.
#addStripe -block_ring_top_layer_limit M3 -max_same_layer_jog_length 0.56 -padcore_ring_bottom_layer_limit M1 -number_of_sets 6 -padcore_ring_top_layer_limit M3 -spacing 2.8 -merge_stripes_value 0.28 -layer M4 -block_ring_bottom_layer_limit M1 -width 2 -nets { vss vdd } -break_stripes_at_block_rings 1
#addStripe -block_ring_top_layer_limit M3 -max_same_layer_jog_length 0.56 -padcore_ring_bottom_layer_limit M1 -number_of_sets 6 -padcore_ring_top_layer_limit M3 -spacing 2.8 -merge_stripes_value 0.28 -layer M4 -block_ring_bottom_layer_limit M1 -width 2 -nets { vss vdd }
#addStripe -block_ring_top_layer_limit M5 -max_same_layer_jog_length 0.6 -padcore_ring_bottom_layer_limit M1 -number_of_sets 6 -padcore_ring_top_layer_limit M4 -spacing 1.84 -merge_stripes_value 0.2 -layer M5 -block_ring_bottom_layer_limit M1 -width 1 -nets { vss vdd } -break_stripes_at_block_rings 1
#addStripe -block_ring_top_layer_limit M7 -max_same_layer_jog_length 0.56 -padcore_ring_bottom_layer_limit M1 -number_of_sets 3 -padcore_ring_top_layer_limit M7 -spacing 0.5 -layer M7 -block_ring_bottom_layer_limit M1 -width 1 -nets { vss vdd }

# Route power nets
# sroute: Special route - vdd/vss wires between rings and core power rails
sroute -nets {vdd vss}
#sroute \
#	-connect {blockPin padPin padRing corePin floatingStripe } \
#	-allowJogging 1 \
#	-allowLayerChange 1 \
#	-blockPin useLef \
#	-targetViaLayerRange {M1 M2}

# It adds the decoupling caps
addEndCap -prefix PwrCap -preCap $env(POWERCAP_CELL) -postCap $env(POWERCAP_CELL) -flipY
addWellTap -cell $env(WELLTAP_CELL) -cellInterval 30 -prefix TAP 

# Avoiding adding antenna diodes initially so that LVS will pass easier. Can change this later when design is more finalized/checked.
setNanoRouteMode -routeInsertAntennaDiode false
setNanoRouteMode -drouteUseViaOfCut 2
setNanoRouteMode -drouteUseMultiCutViaEffort high


# Allow nanoroute to try harder/longer
#setNanoRouteMode -envNumberFailLimit 11

# Place standard cells
setPlaceMode -place_detail_legalization_inst_gap 2
place_design   
refinePlace -checkRoute 0  -preserveRouting 0 -rmAffectedRouting 0 -swapEEQ 0 -checkPinLayerForAccess 1

#sroute

# Add tie hi lo cells
addTieHiLo -cell $env(TIEHILO_CELL) -createHierPort true -reportHierPort true

# Run in-place optimization
# to fix setup problems
optDesign -preCTS
optDesign -preCTS -drv

group_path -name CLOCK -from $env(DESIGN_CLOCK)
# Run Clock Tree Synthesis
#createClockTreeSpec -output encounter.cts
#createClockTreeSpec -output encounter.cts
#specifyClockTree -clkfile encounter.cts(update to next line)
#specifyClockTree -file encounter.cts 
##ckSynthesis -rguide cts.rguide -report report.ctsrpt -macromodel report.ctsmdl -fix_added_buffers

# -- clock tree synthesis ----------------------------------------------------

setCTSMode \
  -traceDPinAsLeaf true \
  -traceIoPinAsLeaf true \
  -addClockRootProp true

set_ccopt_property buffer_cells {BUFX16MA10TR BUFX16BA10TR FRICGX11BA10TR BUFX5BA10TR FRICGX13BA10TR INVX16BA10TR INVX9BA10TR}

ccopt_design

#connectGlobalNets


#extractRC
#reportClockTree -postRoute -localSkew -report skew.postRoute.ctsrpt

# Run in-place optimization
optDesign -postCTS 

# Fix all remaining violations
optDesign -postCTS -drv

reset_path_group -name CLOCK
#clearClockDomains
#setClockDomains -fromType register -toType register
optDesign -postCTS -hold
#clearClockDomains

# Add filler cells
setFillerMode -core $env(FILLER_CELL) -corePrefix FILL -merge true
addFiller

# Run global routing
routeDesign -globalDetail

setNanoRouteMode -droutePostRouteSwapVia true
setNanoRouteMode -drouteStartIteration 20
setNanoRouteMode -drouteEndIteration default

routeDesign -globalDetail


setDelayCalMode -engine aae -SIAware true
setAnalysisMode -analysisType onChipVariation -cppr both

# Final opt
#optDesign -postRoute -si 
optDesign -postRoute

#clearClockDomains
#   setClockDomains -fromType register -toType register
#   setOptMode -considerNonActivePathGroup true
#   optDesign -postroute -hold
#   clearClockDomains
#   setOptMode -considerNonActivePathGroup false

optDesign -postRoute -hold

optDesign -postRoute -drv 

# Export DEF
defOut -routing -floorplan final.def

## Output GDSII
# streamOut final.gds2 -mapFile "/ece498hk/libs/TSMC65GP_RFMIM__1P0V_2P5V__1p9m_6X1Z1U_ALRDL/tsmcN65/tsmcN65.layermap" -libName DesignLib -merge "/ece498hk/libs/TSMC65GP_RFMIM__1P0V_2P5V__1p9m_6X1Z1U_ALRDL/stdcell_dig/fb_tsmc065gp_rvt_lvt/aci/sc-ad10/gds2/tsmc65_rvt_sc_adv10.gds2 libraries/icache_tag_store.gds2 libraries/icache_data_store.gds2 libraries/dmem_sram.gds2" -stripes 1 -mode ALL
#######*********TODO**************###########
#######*********TODO**************###########
#######*********TODO**************###########
streamOut final.gds2 -mapFile "pnr/tsmc065.map" -libName DesignLib -merge "/ece498hk/libs/TSMC65GP_RFMIM__1P0V_2P5V__1p9m_6X1Z1U_ALRDL/stdcell_dig/fb_tsmc065gp_rvt_lvt/aci/sc-ad10/gds2/tsmc65_rvt_sc_adv10.gds2 libraries/icache_tag_store.gds2 libraries/icache_data_store.gds2 libraries/dmem_sram.gds2" -stripes 1 -mode ALL
#######*********TODO**************###########
#######*********TODO**************###########
#######*********TODO**************###########

## Output Verilog Netlist
## -phys option adds power and ground nets .vdd(vdd) etc. into verilog netlist
#saveNetlist $env(VLOGOUT_DIR)/$env(TOP_LEVEL).pnr.v -excludeTopCellPGPort {vdd vss} -excludeLeafCell -excludeCellInst {FILL1 FILL2 FILL4 FILL8 FILL16 FILL32 FILL64}
##saveNetlist $env(VLOGOUT_DIR)/$env(TOP_LEVEL).pnr.v -flat -includePhysicalCell {FILLCAP16A10TR FILLCAP8A10TR} -excludeLeafCell -excludeCellInst {FILL128A10TR FILLTIE128A10TR FILL64A10TR FILLTIE64A10TR FILL32A10TR FILLTIE32A10TR FILL16A10TR FILLTIE16A10TR FILL8A10TR FILLTIE8A10TR FILL4A10TR FILLTIE4A10TR FILL2A10TR FILLTIE2A10TR FILL1A10TR}
#saveNetlist $env(VLOGOUT_DIR)/$env(TOP_LEVEL).pnr.v -excludeLeafCell -excludeCellInst {FILL1A10TR FILLTIE2A10TR}
saveNetlist $env(VLOGOUT_DIR)/$env(TOP_LEVEL).pnr.v -includePhysicalCell $env(INCPHY_CELL) -excludeLeafCell -excludeCellInst $env(EXCPHY_CELL)

##create_library_set -name default_libs_min -timing {$env(TSMC_LIB)}
#create_delay_corner -name default_corner_typ -library_set default_libs_min
#create_constraint_mode -name default_mode_setup -sdc_files [list $env(SDC_OUT_DIR)/$env(TOP_LEVEL).gate.sdc]
#create_analysis_view -name default_view_typ -delay_corner default_corner_typ -constraint_mode default_mode_setup
#set_analysis_view -setup {default_view_setup default_view_typ} -hold {default_view_hold default_view_typ}

write_sdf -version 2.1 -view slowView -min_period_edges posedge $env(SDF_OUT_DIR)/$env(TOP_LEVEL).pnr.sdf

## Output DSPF RC Data
##write_sdf $env(SDF_OUT_DIR)/$env(TOP_LEVEL).pnr_old.sdf
#rcout -spf final.dspf

## Run DRC and Connection checks
verifyGeometry
verifyConnectivity -type all -noAntenna
# verify_drc

saveDesign [format "%s%s" $env(TOP_LEVEL) ".finished.enc"]
#
puts "**************************************"
puts "* Encounter script finished          *"
puts "*                                    *"
puts "* Results:                           *"
puts "* --------                           *"
puts "* DEF File:final.def                 *"
puts "* Layout:  final.gds2                *"
puts "* Netlist: final.v                   *"
puts "* Timing:  timing.rep.5.final        *"
puts "*                                    *"
puts "* Type 'win' to get the Main Window  *"
puts "* or type 'exit' to quit             *"
puts "*                                    *"
puts "**************************************"

#exit
