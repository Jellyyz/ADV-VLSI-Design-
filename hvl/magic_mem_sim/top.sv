module mp4_tb;
`timescale 1ns/10ps

/********************* Do not touch for proper compilation *******************/
// Instantiate Interfaces
tb_itf itf();
rvfi_itf rvfi(itf.clk, itf.rst);

// Instantiate Testbench
source_tb tb(
    .magic_mem_itf(itf),
    .mem_itf(itf),
    .sm_itf(itf),
    .tb_itf(itf),
    .rvfi(rvfi)
);

// For local simulation, add signal for Modelsim to display by default
// Note that this signal does nothing and is not used for anything
bit f;

/****************************** End do not touch *****************************/

/************************ Signals necessary for monitor **********************/
// This section not required until CP2

assign rvfi.commit = 0; // Set high when a valid instruction is modifying regfile or PC
assign rvfi.halt = 0;   // Set high when you detect an infinite loop
initial rvfi.order = 0;
always @(posedge itf.clk iff rvfi.commit) rvfi.order <= rvfi.order + 1; // Modify for OoO

/*
The following signals need to be set:
Instruction and trap:
    rvfi.inst
    rvfi.trap

Regfile:
    rvfi.rs1_addr
    rvfi.rs2_add
    rvfi.rs1_rdata
    rvfi.rs2_rdata
    rvfi.load_regfile
    rvfi.rd_addr
    rvfi.rd_wdata

PC:
    rvfi.pc_rdata
    rvfi.pc_wdata

Memory:
    rvfi.mem_addr
    rvfi.mem_rmask
    rvfi.mem_wmask
    rvfi.mem_rdata
    rvfi.mem_wdata

Please refer to rvfi_itf.sv for more information.
*/

/**************************** End RVFIMON signals ****************************/

/********************* Assign Shadow Memory Signals Here *********************/
// This section not required until CP2
/*
The following signals need to be set:
icache signals:
    itf.inst_read
    itf.inst_addr
    itf.inst_resp
    itf.inst_rdata

dcache signals:
    itf.data_read
    itf.data_write
    itf.data_mbe
    itf.data_addr
    itf.data_wdata
    itf.data_resp
    itf.data_rdata

Please refer to tb_itf.sv for more information.
*/

/*********************** End Shadow Memory Assignments ***********************/

// Set this to the proper value
assign itf.registers = '{default: '0};

/*********************** Instantiate your design here ************************/
/*
The following signals need to be connected to your top level:
Clock and reset signals:
    itf.clk
    itf.rst

Burst Memory Ports:
    itf.mem_read
    itf.mem_write
    itf.mem_wdata
    itf.mem_rdata
    itf.mem_addr
    itf.mem_resp

Please refer to tb_itf.sv for more information.
*/

ibex_top #(
    .PMPEnable        ( 0                                ),
    .PMPGranularity   ( 0                                ),
    .PMPNumRegions    ( 4                                ),
    .MHPMCounterNum   ( 0                                ),
    .MHPMCounterWidth ( 40                               ),
    .RV32E            ( 0                                ),
    .RV32M            ( ibex_pkg::RV32MFast              ),
    .RV32B            ( ibex_pkg::RV32BNone              ),
    .RegFile          ( ibex_pkg::RegFileFF              ),
    .ICache           ( 0                                ),
    .ICacheECC        ( 0                                ),
    .ICacheScramble   ( 0                                ),
    .BranchPrediction ( 0                                ),
    .SecureIbex       ( 0                                ),
    .RndCnstLfsrSeed  ( ibex_pkg::RndCnstLfsrSeedDefault ),
    .RndCnstLfsrPerm  ( ibex_pkg::RndCnstLfsrPermDefault ),
    .DbgTriggerEn     ( 0                                ),
    .DmHaltAddr       ( 32'h1A110800                     ),
    .DmExceptionAddr  ( 32'h1A110808                     )
) dut (
    // Clock and reset
    .clk_i                  (itf.clk),
    .rst_ni                 (itf.rst),
    .test_en_i              (1'b1), //this makes sure that the chip doesnt do any clock gating
    .scan_rst_ni            (1'b1), // document told to tie this off to 1 as we don't have dft
    .ram_cfg_i              (),

    // Configuration
    .hart_id_i              (),
    .boot_addr_i            (),

    // Instruction memory interface
    .instr_req_o            (),
    .instr_gnt_i            (),
    .instr_rvalid_i         (),
    .instr_addr_o           (),
    .instr_rdata_i          (),
    .instr_rdata_intg_i     (),
    .instr_err_i            (),

    // Data memory interface
    .data_req_o             (),
    .data_gnt_i             (),
    .data_rvalid_i          (),
    .data_we_o              (),
    .data_be_o              (),
    .data_addr_o            (),
    .data_wdata_o           (),
    .data_wdata_intg_o      (),
    .data_rdata_i           (),
    .data_rdata_intg_i      (),
    .data_err_i             (),

    // Interrupt inputs
    .irq_software_i         (),
    .irq_timer_i            (),
    .irq_external_i         (),
    .irq_fast_i             (),
    .irq_nm_i               (),

    // Debug interface
    .debug_req_i            (),
    .crash_dump_o           (),

    // Special control signals
    .fetch_enable_i         (),
    .alert_minor_o          (),
    .alert_major_internal_o (),
    .alert_major_bus_o      (),
    .core_sleep_o           ()
);
/***************************** End Instantiation *****************************/

endmodule
